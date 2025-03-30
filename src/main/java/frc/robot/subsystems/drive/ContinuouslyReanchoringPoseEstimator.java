package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Optional;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

public class ContinuouslyReanchoringPoseEstimator extends SwerveDrivePoseEstimator {



    public Pose2d anchorOdometry = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public Pose2d anchorPoseEstimator = new Pose2d(0, 0, Rotation2d.fromDegrees(0));


    public ContinuouslyReanchoringPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
                                                Pose2d initialPose) {
        super(kinematics,gyroAngle, modulePositions, initialPose);
    }


//    public void updateWithTime(double sampleTimestamps, Rotation2d rawGyroRotation, SwerveModulePosition[] modulePositions) {
//        odometry.update(rawGyroRotation, modulePositions);
//        newOdometryEntry(odometry.getPoseMeters());
//    }
//
//    public void newOdometryEntry(Pose2d pose) {
//        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(Timer.getFPGATimestamp(), pose);
//        odometryHistory.add(0, newEntry);
//        if (odometryHistory.size() > 100) {
//            odometryHistory.remove(odometryHistory.size() - 1);
//        }
//    }
//
//
//
//    public void newVisionEntry(Pose2d visionPose, double visionTime) {
////        Debug.println("Vision entry: ", visionTime, " System time: ", System.currentTimeMillis());
//        int matching = 0;
//        for (int i = 0; i < odometryHistory.size(); i++) {
//            if (odometryHistory.get(i).time <= visionTime) {
//                matching = i;
//                break;
//            }
//        }
//        if (matching == 0) {
//            return;
//        }
//
//        var matchingOdometry = odometryHistory.get(matching);
//        double speedDelta = calculateSpeedDelta(visionPose, odometryHistory.get(matching), visionHistory.get(0));
//        visionHistory.add(0,
//                new VisionHistoryEntry(visionPose, odometryHistory.get(matching), speedDelta));
//
//        double totalDiff = 0;
//        if (visionHistory.size() > 10) {
//            for (int i = 0; i < 10; i++) {
//                totalDiff += visionHistory.get(i).speedDiff;
//            }
////            Debug.println("TotalDiff ", totalDiff);
//            if (totalDiff < 50) {
//                anchorOdometry = matchingOdometry.pose;
//                anchorVision = visionPose;
//                diffR = matchingOdometry.pose.getRotation().minus(visionPose.getRotation());
//            }
//        }
//    }
//
//    /* Generate a number representing how far off the vision speed was from the odometry speed */
//     public Pose2d getAnchor() {
//        return anchorVision;
//    }
//
//    public Pose2d getCurrentPose() {
//        OdometryHistoryEntry currentOdometry = odometryHistory.get(0);
//        Transform2d movement = currentOdometry.pose.minus(anchorOdometry);
//        Pose2d newPosition = anchorVision.transformBy(movement);
//        // Debug.debugPrint("posemath", " A:" + anchorVision.toString() + " MV:" +
//        // movement.toString() + " NP:" + newPosition.toString() );
//        return (newPosition);
//    }
//
//    public Pose2d getEstimatedPosition() {
//        return getCurrentPose();
//    }
//
//    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d pose) {
//        odometry.resetPosition(gyroAngle, wheelPositions, pose);
//        anchorOdometry = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//        anchorVision = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//        diffR = Rotation2d.fromDegrees(0);
//        final OdometryHistoryEntry newEntry = new OdometryHistoryEntry(System.currentTimeMillis(), pose);
//        odometryHistory.clear();
//        odometryHistory.add(0, newEntry);
//        visionHistory.clear();
//        visionHistory.add(0, new VisionHistoryEntry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), newEntry, 0));
//    }

}
