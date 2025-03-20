// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {


    public static class DriveConstants {

    }
    public static class SwerveDrivetrain {
        // Physical Attributes
        public static final double WHEELBASE_IN_METERS = 0.56515;
        public static final double DRIVEBASE_RADIUS_IN_METERS =
            (WHEELBASE_IN_METERS / 2) * Math.sqrt(2);
        public static final double WHEEL_RADIUS_IN_METERS = 0.044;
        public static final double DRIVE_GEAR_RATIO = (1.0 / 5.60);
        public static final double WHEEL_CIRCUMFERENCE_IN_METERS =
            WHEEL_RADIUS_IN_METERS * (Math.PI * 2);
        public static final boolean TURNING_MOTORS_INVERTED = true;
        public static final boolean DRIVE_MOTORS_INVERTED = false;

        // Control flags
        public static final boolean BRAKING_DURING_AUTONOMOUS = true;
        public static final boolean BRAKING_DURING_TELEOP = false;

        // Control Tuning
        public static final double CONTROLLER_DEADZONE = 0.1;
        public static final double CONTROLLER_CUBIC_LINEARITY = 0.4;

        // Robot speed and acceleration limiters
        public static final double MOVEMENT_MAX_SPEED_IN_METERS_PER_SECOND = 6.0; // Max ~3.0
        public static final double MOVEMENT_MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED = 3.0; // 1
                                                                                                 // second
                                                                                                 // to
                                                                                                 // full
                                                                                                 // speed
        public static final double TURNING_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND =
            Math.toRadians(360); // 1 rotation per second
        public static final double TURNING_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED =
            Math.toRadians(720); // one half second to full turn speed

        // Wheel rotation speed and acceleration limiters
        public static final double WHEEL_MAX_ANGULAR_VELOCITY_IN_RADIANS_PER_SECOND_SQUARED =
            Math.PI * 2;
        public static final double WHEEL_MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED =
            Math.PI * 200;

        public static class Ports {
            public static final int BACK_LEFT_DRIVE_MOTOR = 13;
            public static final int BACK_LEFT_ROTATION_MOTOR = 23;
            public static final int BACK_LEFT_ROTATION_ENCODER = 33;

            public static final int FRONT_LEFT_DRIVE_MOTOR = 14;
            public static final int FRONT_LEFT_ROTATION_MOTOR = 24;
            public static final int FRONT_LEFT_ROTATION_ENCODER = 34;

            public static final int FRONT_RIGHT_DRIVE_MOTOR = 11;
            public static final int FRONT_RIGHT_ROTATION_MOTOR = 21;
            public static final int FRONT_RIGHT_ROTATION_ENCODER = 31;

            public static final int BACK_RIGHT_DRIVE_MOTOR = 12;
            public static final int BACK_RIGHT_ROTATION_MOTOR = 22;
            public static final int BACK_RIGHT_ROTATION_ENCODER = 32;


            // public static final int BACK_LEFT_DRIVE_MOTOR = 11;
            // public static final int BACK_LEFT_ROTATION_MOTOR = 21;
            // public static final int BACK_LEFT_ROTATION_ENCODER = 31;
            //
            // public static final int FRONT_LEFT_DRIVE_MOTOR = 12;
            // public static final int FRONT_LEFT_ROTATION_MOTOR = 22;
            // public static final int FRONT_LEFT_ROTATION_ENCODER = 32;
            //
            // public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
            // public static final int FRONT_RIGHT_ROTATION_MOTOR = 23;
            // public static final int FRONT_RIGHT_ROTATION_ENCODER = 33;
            //
            // public static final int BACK_RIGHT_DRIVE_MOTOR = 14;
            // public static final int BACK_RIGHT_ROTATION_MOTOR = 24;
            // public static final int BACK_RIGHT_ROTATION_ENCODER = 34;
        }
    }

    public static class Elevator {
        public static double[] States = {0, 13, 22, 38, 62};

        public static final TrapezoidProfile.Constraints ELEVATOR_TRAPEZOID_PROFILE =
            new TrapezoidProfile.Constraints(300.0, 100.0);

        public static final double KG = 0.15;
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final boolean ENCODER_REVERSED = false;
        public static final Encoder.EncodingType ENCODER_ENCODING_TYPE = Encoder.EncodingType.k4X;
        public static final double MANUAL_ADJUST_STEP = 0.5;

        public static class Ports {
            public static final int MOTOR_1 = 8;
            public static final int MOTOR_2 = 9;
            public static final int ENCODER_CHANNEL_A = 1;
            public static final int ENCODER_CHANNEL_B = 2;
        }
    }

    public static class Pickup {
        public static boolean MOTOR_1_INVERTED = false;
        public static boolean MOTOR_2_INVERTED = true;

        public static int CURRENT_LIMIT_STALLED = 20;
        public static int CURRENT_LIMIT_FREE = 20;

        public static double INTAKE_IN_SPEED = 1;
        public static double INTAKE_OUT_SPEED = -1;

        public static double RAISED_SETPOINT = 2.5;
        public static double LOWERED_SETPOINT = 8;

        public static final double KP = 0.05;
        public static final double KI = 0;
        public static final double KD = 0;

        public static class Ports {
            public static final int ROTATION_MOTOR = 10;
            public static final int DELIVERY_MOTOR_ID = 5;

        }
    }

    public static class Delivery {
        public static int CURRENT_LIMIT_STALLED = 10;
        public static int CURRENT_LIMIT_FREE = 15;
        public static final double DELIVERY_OUT_SPEED = -1;
        public static final double DELIVERY_IN_SPEED = 1;

        public static class Ports {
            public static final int CONVEYOR_MOTOR_ID = 6;
        }

        public static final boolean DELIVERY_MOTOR_INVERTED = false;
        public static boolean CONVEYOR_MOTOR_INVERTED = false;
        public static final double CONVEYOR_OUT_SPEED = -0.5;
        public static final double CONVEYOR_IN_SPEED = 0.5;
    }

    public static final class Climber {
        public static final double KP = 1;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double CLIMBING_SETPOINT = 0;
        public static final double DOCKING_SETPOINT = 0;

        public static class Ports {
            public static final int CLIMBER_MOTOR = 7;
        }
    }

//    public static final class VisionSubsystem {
//        public static final String ALIGNMENT_CAMERA_NAME_CLOSE = "ALIGNMENT_CAMERA_CLOSE";
//        public static final Transform3d CAMERA_TO_ROBOT_TRANSFORM =
//            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
//    }


    public static double loopPeriodSecs = 0.02;

    // Use LoggedTunableNumbers
    public static final boolean tuningMode = true;

    private static RobotType robotType = RobotType.BAJA;

    public static RobotType getRobot()
    {
        return robotType;
    }

    /**
     * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when
     * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
     * "replay" (log replay from a file).
     */
    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        GORT,
        BAJA
    }
}


