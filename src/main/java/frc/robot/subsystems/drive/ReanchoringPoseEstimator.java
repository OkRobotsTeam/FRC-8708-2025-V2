package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Optional;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;
import org.littletonrobotics.junction.Logger;


public class ReanchoringPoseEstimator {

    /* A simple pair of poses and the times those poses were recorded */
    private class OdometryHistoryEntry {
        public double time;
        public Pose2d pose;

        OdometryHistoryEntry(double time, Pose2d pose) {
            this.time = time;
            this.pose = pose;
        }
    }

    /*
     * VisionHistoryEntry is the most important data structure in the reanchoring algorithm. It contains a pose2d of where vision
     * thinks the robot is combined with a matching odometry history entry and speed difference.
     *
     * Finding the matching odometry history entry requires looking through the odometry history for a measurement that
     * was taken at approximately the same time as the snapshot was taken that the vision positioning is based on.
     *
     * The third entry in the data structure is a speed difference. This is calculated by generating the speed indicated by vision
     * (comparing the positions indicated by the last two vision entries) and the speed indicated by odometry and differencing those
     * speeds. It is used to determine confidence that the vision entry is valid and accurate. Inaccurate vision measurements tend
     * to bounce around rapidly and their speeds do not match the speed of the robot.
     *
     * With this data structure populated, we can anchor our odometry to the most recent accurate vision reading in the past.
     */
    private class VisionHistoryEntry {
        public OdometryHistoryEntry o;
        public Pose2d pose;
        public double speedDiff;

        public VisionHistoryEntry(Pose2d pose, OdometryHistoryEntry o, double speedDiff) {
            this.pose = pose;
            this.o = o;
            this.speedDiff = speedDiff;
        }
    }
    public Pose2d zeroPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public Pose2d anchorOdometry = zeroPose;
    public Pose2d anchorVision = zeroPose;
    public Rotation2d diffR = Rotation2d.fromDegrees(0);
    public Pose2d anchorOdometry2 = zeroPose;
    public Pose2d anchorVision2 = zeroPose;
    public Rotation2d diffR2 = Rotation2d.fromDegrees(0);

    public SwerveDriveOdometry odometry;
    ArrayList<OdometryHistoryEntry> odometryHistory = new ArrayList<OdometryHistoryEntry>();
    ArrayList<VisionHistoryEntry> visionHistory = new ArrayList<VisionHistoryEntry>();

    public ReanchoringPoseEstimator() {
        odometryHistory.add(0, new OdometryHistoryEntry(Timer.getFPGATimestamp(), new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        visionHistory.add(0, new VisionHistoryEntry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), odometryHistory.get(0), 0));
    }

    public ReanchoringPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
            Pose2d initialPose) {
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPose);
        odometryHistory.add(0, new OdometryHistoryEntry(Timer.getFPGATimestamp(), new Pose2d(0, 0, gyroAngle)));
        visionHistory.add(0, new VisionHistoryEntry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), odometryHistory.get(0), 0));
    }

    public void updateWithTime(double sampleTimestamps, Rotation2d rawGyroRotation, SwerveModulePosition[] modulePositions) {
        odometry.update(rawGyroRotation, modulePositions);
        newOdometryEntry(odometry.getPoseMeters());
    }

    public void newOdometryEntry(Pose2d pose) {
        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(Timer.getFPGATimestamp(), pose);
        odometryHistory.add(0, newEntry);
        if (odometryHistory.size() > 100) {
            odometryHistory.remove(odometryHistory.size() - 1);
        }
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        //Debug.dprintln("Vision Times:", "FPGATime:", Timer.getFPGATimestamp(), "Vision:", timestampSeconds);
        newVisionEntry(visionRobotPoseMeters, timestampSeconds);
    }

    public void newVisionEntry(Pose2d visionPose, double visionTime) {
//        Debug.println("Vision entry: ", visionTime, " System time: ", System.currentTimeMillis());
        int matching = -1;
        Logger.recordOutput("ReanchoringPoseEstimator/VisionPose", visionPose);
        for (int i = 0; i < odometryHistory.size(); i++) {
            if (odometryHistory.get(i).time <= visionTime) {
                matching = i;
                break;
            }
        }
        if (matching == -1) {
            System.out.println("No matching odometry entry found for vison measurement.  This shouldn't happen.");
            return;
        }
        Logger.recordOutput("ReanchoringPoseEstimator/VisionLatency", Timer.getFPGATimestamp() - visionTime);
        var matchingOdometry = odometryHistory.get(matching);
        double speedDelta = calculateSpeedDelta(visionPose, odometryHistory.get(matching), visionHistory.get(0));
        visionHistory.add(0,
                new VisionHistoryEntry(visionPose, odometryHistory.get(matching), speedDelta));

        double totalDiff = 0;
        if (visionHistory.size() > 2) {
            int sampleSize = Math.min(10,visionHistory.size());
            for (int i = 0; i < sampleSize; i++) {
                totalDiff += visionHistory.get(i).speedDiff;
            }
//            Debug.println("TotalDiff ", totalDiff);
            Logger.recordOutput("ReanchoringPoseEstimator/TotalDiff", totalDiff);

            if (totalDiff < 50) {
                //Re-Anchor
                anchorOdometry = matchingOdometry.pose;
                anchorVision = visionPose;
                diffR = matchingOdometry.pose.getRotation().minus(anchorVision.getRotation());
                Logger.recordOutput("ReanchoringPoseEstimator/VisionAnchor", anchorVision);
                Logger.recordOutput("ReanchoringPoseEstimator/OdometryAnchor", anchorOdometry);

                //To-do
                /* the reanchoring algorythm should be rewritten to assume that vision can only pinpoint the robots position
                within a circle of a radius that grows proportional to the distance from the tag it is detecting.  The further the
                robot from the tag, the bigger the circle of inaccuracy.  It should only correct the anchor if the vision measurement
                is outside that circle.  Using this method, as the robot moves closer to a tag, the position would be continuously
                adjusted to be more accurate.  The data would still be used, but only outliers in one direction which would pull the
                anchor twoards that direction using interpolation.  If the circle is slightly bigger than the noise profile, the anchor
                will only get pushed towards the right place.

                It should also use this circle to dampen movement detected by vision.  Currently the speedDiff is registering lots
                of movement from vision when the robot is still even though the vision measurements are relatively stable because
                of how fast the robot would have to move to actually be at all the positions indicated.  This causes us to consider
                data unusable that we could probably be using to our advantage.  If we could ignore motion within the circle of
                inaccuracy then we could better determine if the data is good.  I haven't figured out how to do this math yet.  It might
                be better to simply accept all the data and use the radius of innacuracy to choose not to reanchor as long as we are
                within that circle.  The size of the circle could be a tunable parameter that we get from looking at AdvantageScope replays
                (Assuming we ever figure those out)*/
            }
        }
        //Testing alternate algorithm
        if (visionHistory.size() > 0) {
            int lookBack = Math.min(5, visionHistory.size());
            double oneSecondAgo = Timer.getFPGATimestamp() - 1;
            int relevantSamples = 1;
            for (int i=0; i<lookBack; i++)  {
                if (visionHistory.get(i).o.time > oneSecondAgo) {
                    relevantSamples++;
                }
            }
            anchorOdometry2 = matchingOdometry.pose;
            Pose2d oldEstimate = getEstimatedPosition(matchingOdometry.pose);
            Pose2d newEstimate = oldEstimate.interpolate( visionPose, 1.0/relevantSamples );
            diffR2 = matchingOdometry.pose.getRotation().minus(anchorVision2.getRotation());
            anchorVision2 = newEstimate;
        } else {
            anchorOdometry2 = matchingOdometry.pose;
            anchorVision2 = visionPose;
            diffR2 = matchingOdometry.pose.getRotation().minus(anchorVision2.getRotation());
        }
    }



    /* Generate a number representing how far off the vision speed was from the odometry speed */
    public double calculateSpeedDelta(Pose2d visionPose, OdometryHistoryEntry thisOdometry,
            VisionHistoryEntry lastVision) {
        double visionMovement = distanceBetween(visionPose, lastVision.pose);
        double visionRotation = Math.abs(visionPose.getRotation().minus(lastVision.pose.getRotation()).getRadians());
        double visionCombined = (visionMovement + visionRotation * 5) * 100;
        // Debug.debugPrint("Vision Math", "Lateral: " + fmt(visionMovement) + "
        // Rotation:" +
        // fmt(visionRotation) + " vx: " + fmt(visionPose.getX()) + " vy: " +
        // fmt(visionPose.getY()));
        OdometryHistoryEntry lastOdometry = lastVision.o;
        double odometryMovement = distanceBetween(thisOdometry.pose, lastOdometry.pose);
        double odometryRotation = Math.abs(
                thisOdometry.pose.getRotation().minus(lastOdometry.pose.getRotation()).getRadians());
        double odometryCombined = (odometryMovement + odometryRotation * 5) * 100;
//        Debug.println("speedDiff", "Vision Movement: ", visionCombined, " Odometry Movement: ",
//                odometryCombined);
        return (Math.abs(visionCombined - odometryCombined));
    }

    public double distanceBetween(Pose2d pose1, Pose2d pose2) {
        return pose1.getTranslation().getDistance(pose2.getTranslation());
    }

    public Pose2d getRobotPose() {
        return getCurrentPose();
    }

    public Pose2d getAnchor() {
        return anchorVision;
    }

    public Pose2d getEstimatedPosition(Pose2d odometryPose) {
        Transform2d movement = odometryPose.minus(anchorOdometry);
        return(anchorVision.transformBy(movement));
    }

    public Pose2d getCurrentPose() {
        OdometryHistoryEntry currentOdometry = odometryHistory.get(0);
        Transform2d movement = currentOdometry.pose.minus(anchorOdometry);
        Pose2d newPosition = anchorVision.transformBy(movement);
        // Debug.debugPrint("posemath", " A:" + anchorVision.toString() + " MV:" +
        // movement.toString() + " NP:" + newPosition.toString() );
        return (newPosition);
    }

    public Pose2d getEstimatedPosition2() {
        OdometryHistoryEntry currentOdometry = odometryHistory.get(0);
        Transform2d movement = currentOdometry.pose.minus(anchorOdometry2);
        return anchorVision2.transformBy(movement);
    }

    public Pose2d getEstimatedPosition() {
        return getCurrentPose();
    }

    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d pose) {
        odometry.resetPosition(gyroAngle, wheelPositions, pose);
        anchorOdometry = zeroPose;
        anchorVision = zeroPose;
        diffR = Rotation2d.fromDegrees(0);
        anchorOdometry2 = zeroPose;
        anchorVision2 = zeroPose;
        diffR2 = Rotation2d.fromDegrees(0);
        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(System.currentTimeMillis(), pose);
        odometryHistory.clear();
        odometryHistory.add(0, newEntry);
        visionHistory.clear();
        visionHistory.add(0, new VisionHistoryEntry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), newEntry, 0));
    }

}
