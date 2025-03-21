// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Vision;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout;
    private static boolean usedCustomField = false;
    static {
        try {
            aprilTagLayout =
                new AprilTagFieldLayout(Path
                    .of(Filesystem.getDeployDirectory().getAbsolutePath()
                        + "/vision/andymark.json"));
            usedCustomField = true;
        } catch (Exception e) {
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }
        Logger.recordOutput("Used Custom Field?", usedCustomField);
    }

    // Camera names, must match names configured on coprocessor
    public static String alignmentCameraName = "alignment_camera";
//    public static String frontCameraName = "front_camera";

    // Robot to camera transforms
//    public static Transform3d robotToAlignmentCamera =
//            new Transform3d(
//                    Units.inchesToMeters(5), // +Forwards/ -Backwards  3.5
//                    Units.inchesToMeters(12), // +Right / -Left  11.5
//                    Units.inchesToMeters(31.25), // +Up / -Down
//                    new Rotation3d(
//                            Units.degreesToRadians(-37.5), // 45
//                            Units.degreesToRadians(180),
//                            Units.degreesToRadians(-90)) // -90
//
//            );

    public static Transform3d robotToAlignmentCamera =
            new Transform3d(
                    Units.inchesToMeters(3), // +Forwards/ -Backwards  3.5
                    Units.inchesToMeters(-14), // +Right / -Left  11.5
                    Units.inchesToMeters(12), // +Up / -Down
                    new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(-90))

            );

//    public static Transform3d robotToFrontCamera =
//            new Transform3d(
//                    Units.inchesToMeters(0), // +Forwards/ -Backwards  3.5
//                    Units.inchesToMeters(0), // +Right / -Left  11.5
//                    Units.inchesToMeters(0), // +Up / -Down
//                    new Rotation3d(
//                            Units.degreesToRadians(0), // 45
//                            Units.degreesToRadians(0),
//                            Units.degreesToRadians(0)) // -90
//
//            );

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
                1.0,
                1.0,
        };

//    public static List<Integer> rejectedTags = Arrays.asList(2, 3, 4, 5, 14, 15, 16);
        public static List<Integer> reefTags = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    public static List<Integer> rejectedTags = List.of();
}
