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
            aprilTagLayout = new AprilTagFieldLayout(Path
                    .of(Filesystem.getDeployDirectory().getAbsolutePath()
                            + "/vision/andymark.json"));
            usedCustomField = true;
        } catch (Exception e) {
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            System.out.println("ERROR: FAILED TO LOAD FIRST JSON");
            System.out.println("FALLING BACK TO k2025ReefscapeAndyMark");
        }
        Logger.recordOutput("Used Custom Field?", usedCustomField);
    }

    // Camera names, must match names configured on coprocessor
    public static String centerAlignmentCameraName = "center_alignment_camera";
    public static String leftAlignmentCameraName = "left_alignment_camera";
    public static String rightAlignmentCameraName = "right_alignment_camera";

    public static Transform3d robotToCenterAlignmentCamera = new Transform3d(
            Units.inchesToMeters(1.983), // +Forwards/ -Backwards 3.5
            Units.inchesToMeters(-13.292), // +Right / -Left 11.5
            Units.inchesToMeters(8.262), // +Up / -Down
            new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-90))
    );

    public static Transform3d robotToLeftAlignmentCamera = new Transform3d(
            Units.inchesToMeters(13.034), // +Forwards/ -Backwards 3.5
            Units.inchesToMeters(-11.956), // +Right / -Left 11.5
            Units.inchesToMeters(6.289), // +Up / -Down
            new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(15),
                    Units.degreesToRadians(-123.62)
            )
    );

    public static Transform3d robotToRightAlignmentCamera = new Transform3d(
            Units.inchesToMeters(-12.86), // +Forwards/ -Backwards 3.5
            Units.inchesToMeters(-11.929), // +Right / -Left 11.5
            Units.inchesToMeters(6.237), // +Up / -Down
            new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(15),
                    Units.degreesToRadians(-51.318)
            )
    );

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.4;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
//    public static double linearStdDevBaseline = 0.2; // Meters
//    public static double angularStdDevBaseline = 0.6; // Radians


    public static double linearStdDevBaseline = 0.25; // Meters
    public static double angularStdDevBaseline = 1000000.0; // Radians


    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[]{
            1.0,
            1.0,
            1.0
    };

    public static List<Integer> rejectedTags = Arrays.asList(1, 2, 3, 4, 5, 12, 13, 14, 15, 16);
    public static List<Integer> reefTags = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    //public static List<Integer> rejectedTags = List.of();

    public static double LEFT_ALIGNMENT_OFFSET_INCHES = 0;
    public static double RIGHT_ALIGNMENT_OFFSET_INCHES = 17;
}
