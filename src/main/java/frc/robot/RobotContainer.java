// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Delivery.CONVEYOR_IN_SPEED;
import static frc.robot.Constants.Delivery.CONVEYOR_OUT_SPEED;
import static frc.robot.commands.DriveCommands.oldJoystickApproach;
import static frc.robot.subsystems.Vision.VisionConstants.*;
import static java.lang.Math.abs;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Elastic;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);
    private final CommandGenericHID elevatorButtons = new CommandGenericHID(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<String> autoLineUp;
    private final LoggedDashboardChooser<Double> speedChooser;
    private final LoggedDashboardChooser<String> visionMode;


    // AK-enabled Subsystems
    public final Drive swerveDrivetrain;

    private final Delivery delivery = new Delivery();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Pickup pickup = new Pickup();

    public final Vision vision;
    public String visionModeString = "Simple";

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        swerveDrivetrain = new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new Vision(swerveDrivetrain,
                new VisionIOPhotonVision(centerAlignmentCameraName, robotToCenterAlignmentCamera));
//                new VisionIOPhotonVision(leftAlignmentCameraName, robotToLeftAlignmentCamera),
//                new VisionIOPhotonVision(rightAlignmentCameraName, robotToRightAlignmentCamera));

        // Logic Triggers
        registerNamedCommands();


        autoLineUp = new LoggedDashboardChooser<>("Auto Line Up");
        speedChooser = new LoggedDashboardChooser<>("Drive Speed");

        autoLineUp.addDefaultOption("Old Line Up", "Old Line Up");
        speedChooser.addDefaultOption("100%", 1.0);
        speedChooser.addOption("75%", 0.75);
        speedChooser.addOption("50%", 0.50);
        speedChooser.addOption("25%", 0.25);
        autoLineUp.addOption("New Line Up", "New Line Up");
        // Set up auto routines
        visionMode = new LoggedDashboardChooser<>("Vision Mode");
        visionMode.addDefaultOption("Simple", "Simple");
        visionMode.addOption("Anchoring","Anchoring");
        visionMode.addOption("None","None");


        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(swerveDrivetrain));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerveDrivetrain));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                swerveDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                swerveDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", swerveDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", swerveDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the controller button and joystick bindings
        configureControllerBindings();
//        configureControllerBindingsDemoTwoControllers();
//        configureControllerBindingsParade();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void init() {
        elevator.transitionToState(0);
        delivery.setDeliveryMotor(0);
        pickup.foldPickup();
        swerveDrivetrain.setSpeed(1.0);
        vision.visionEnabled = true;
    }

    public void teleopInit() {
        delivery.setDefaultCommand(Commands.run(() -> {
            if (Math.abs(manipulatorController.getHID().getRightTriggerAxis()) > 0.55) {
                delivery.setDeliveryMotor(
                        (-manipulatorController.getHID().getRightTriggerAxis() / 4.0));
            }
            if (Math.abs(driveController.getHID().getRightTriggerAxis()) > 0.55) {
                delivery.setDeliveryMotor(
                        (-driveController.getHID().getRightTriggerAxis() / 4.0));
            }
        }, delivery));


        pickup.raisePickup();
    }

    public void periodic() {
        SmartDashboard.putNumber("EncoderPositionInches", elevator.getElevatorPosition());
    }
    public void teleopPeriodic() {
        if (Math.abs(manipulatorController.getHID().getRightY()) > 0.2) {
            pickup.manualAdjust(-manipulatorController.getHID().getRightY() * 0.16);
        }

        double manualAdjustAmount = manipulatorController.getHID().getLeftY();
        if (abs(manualAdjustAmount) < 0.2) {
            manualAdjustAmount = 0;
        }

        elevator.manualAdjust(manualAdjustAmount * 6);

        if (!visionMode.get().equals(visionModeString)) {
            visionModeString = visionMode.get();
            System.out.println("Changing vision mode to " + visionModeString);
            if (visionModeString.equals("None")) {
                swerveDrivetrain.setPoseEstimator(Drive.VisionMode.NONE);
            } else if (visionModeString.equals("Anchoring")) {
                swerveDrivetrain.setPoseEstimator(Drive.VisionMode.ANCHORING);
            } else {
                swerveDrivetrain.setPoseEstimator(Drive.VisionMode.SIMPLE);
            }
        }
    }

    public void testInit() {
        teleopInit();
        swerveDrivetrain.setSpeed(0.1);
    }

    public void testPeriodic() {
        teleopPeriodic();

    }

    private Command joystickDrive() {
        return DriveCommands.joystickDrive(
                driveController,
                swerveDrivetrain,
                () -> -driveController.getLeftY() * speedChooser.get(),
                () -> -driveController.getLeftX() * speedChooser.get(),
                () -> -driveController.getRightX() * speedChooser.get(),
                () -> -driveController.getLeftTriggerAxis(),
                elevator::getElevatorPosition,
                driveController.x());
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose) {
        return DriveCommands.joystickApproach(
                swerveDrivetrain,
                () -> -driveController.getLeftY(),
                approachPose);
    }

    public Pose2d getPoseOfNearestReefBranch(FieldConstants.ReefSide side) {
        Pose2d currentPose = swerveDrivetrain.getPose();
        List<AprilTag> aprilTags = VisionConstants.aprilTagLayout.getTags();
        AprilTag closest = aprilTags.get(0);
        double bestDistance = Double.POSITIVE_INFINITY;

        for (AprilTag tag : aprilTags) {
            if (!VisionConstants.reefTags.contains(tag.ID)) {
                continue;
            }

            double currentDistance = tag.pose.getTranslation().toTranslation2d().getDistance(currentPose.getTranslation());

            if (currentDistance < bestDistance) {
                bestDistance = currentDistance;
                closest = tag;
            }
        }

        Pose2d nearestTagPose = closest.pose.toPose2d();

        Translation2d transformToBranch = new Translation2d();

        if (side == FieldConstants.ReefSide.LEFT) {
            transformToBranch = new Translation2d(Units.inchesToMeters(2), nearestTagPose.getRotation().plus(Rotation2d.fromDegrees(90)));
        } else if (side == FieldConstants.ReefSide.RIGHT) {
            transformToBranch = new Translation2d(Units.inchesToMeters(16), nearestTagPose.getRotation().plus(Rotation2d.fromDegrees(90)));
        }

        Translation2d nearestTagTranslation = nearestTagPose.getTranslation().plus(transformToBranch);

        System.out.println("Closest reef tag ID: " + closest.ID);
        System.out.println("Closest reef tag pose: " + nearestTagTranslation);
        return new Pose2d(nearestTagTranslation, nearestTagPose.getRotation());
    }

    /**
     * Button and Command mappings
     */
    private void configureControllerBindings() {
        // Default command, normal field-relative drive
        swerveDrivetrain.setDefaultCommand(joystickDrive());
        driveController.a().onTrue(
                Commands.runOnce(() -> swerveDrivetrain.setPose(new Pose2d())));

//         Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        driveController.rightBumper().and(() -> Objects.equals(autoLineUp.get(), "New Line Up"))
                .whileTrue(
                        joystickApproach(
                                () -> getPoseOfNearestReefBranch(FieldConstants.ReefSide.RIGHT)
                        )
                );

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        driveController.leftBumper().and(() -> Objects.equals(autoLineUp.get(), "New Line Up"))
                .whileTrue(
                        joystickApproach(
                                () -> getPoseOfNearestReefBranch(FieldConstants.ReefSide.LEFT)
                        )
                );


        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        driveController.rightBumper().and(() -> Objects.equals(autoLineUp.get(), "Old Line Up"))
                .whileTrue(
                        oldJoystickApproach(
                                swerveDrivetrain,
                                driveController::getLeftY,
                                driveController::getLeftX,
                                elevator::getElevatorPosition,
                                () -> driveController.x().getAsBoolean(),
                                () -> true));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        driveController.leftBumper().and(() -> Objects.equals(autoLineUp.get(), "Old Line Up"))
                .whileTrue(
                        oldJoystickApproach(
                                swerveDrivetrain,
                                driveController::getLeftY,
                                driveController::getLeftX,
                                elevator::getElevatorPosition,
                                () -> driveController.x().getAsBoolean(),
                                () -> false));


//
        manipulatorController.povUp().onTrue(Commands.runOnce(elevator::nextState));
        manipulatorController.povDown().onTrue(Commands.runOnce(elevator::previousState));

        manipulatorController.leftBumper().and(manipulatorController.rightBumper().negate())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.rightBumper().and(manipulatorController.leftBumper().negate())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.leftBumper().and(manipulatorController.rightBumper())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(1.0)));
        manipulatorController.leftBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));
        manipulatorController.rightBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));

        manipulatorController.leftTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_IN_SPEED)));
        manipulatorController.leftTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        manipulatorController.rightTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));
        driveController.rightTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        manipulatorController.y().onTrue(Commands.runOnce(elevator::toggleHigh));

        manipulatorController.a().onTrue(Commands.runOnce(pickup::lowerPickup));
        manipulatorController.a().onFalse(Commands.runOnce(pickup::raisePickup));

        manipulatorController.b().onTrue(Commands.runOnce(pickup::runIntakeOut));
        manipulatorController.b().onFalse(Commands.runOnce(pickup::stopIntake));

        manipulatorController.x().onTrue(Commands.runOnce(pickup::togglePickup));

        driveController.a().onTrue(Commands.runOnce(() -> pickup.raisePickupToPosition(2.0)));

        manipulatorController.povLeft().onTrue(Commands.runOnce(pickup::middlePickup));

        elevatorButtons.button(1).onTrue(Commands.runOnce(() -> elevator.transitionToState(4)));
        elevatorButtons.button(2).onTrue(Commands.runOnce(() -> elevator.transitionToState(3)));
        elevatorButtons.button(3).onTrue(Commands.runOnce(() -> elevator.transitionToState(2)));
        elevatorButtons.button(4).onTrue(Commands.runOnce(() -> elevator.transitionToState(1)));
        elevatorButtons.button(5).onTrue(Commands.runOnce(() -> elevator.transitionToState(0)));


        driveController.leftTrigger().onTrue(Commands.runOnce(
                () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)).andThen(
                Commands.waitSeconds(0.5).andThen(
                        () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))));

    }

    private void configureControllerBindingsDemoTwoControllers() {
        // Default command, normal field-relative drive
        swerveDrivetrain.setDefaultCommand(joystickDrive());
        driveController.a().onTrue(
                Commands.runOnce(() -> swerveDrivetrain.setPose(new Pose2d())));

//         Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        driveController.rightBumper().and(() -> Objects.equals(autoLineUp.get(), "New Line Up"))
                .whileTrue(
                        joystickApproach(
                                () -> getPoseOfNearestReefBranch(FieldConstants.ReefSide.RIGHT)
                        )
                );

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        driveController.leftBumper().and(() -> Objects.equals(autoLineUp.get(), "New Line Up"))
                .whileTrue(
                        joystickApproach(
                                () -> getPoseOfNearestReefBranch(FieldConstants.ReefSide.LEFT)
                        )
                );


        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        driveController.rightBumper().and(() -> Objects.equals(autoLineUp.get(), "Old Line Up"))
                .whileTrue(
                        oldJoystickApproach(
                                swerveDrivetrain,
                                driveController::getLeftY,
                                driveController::getLeftX,
                                elevator::getElevatorPosition,
                                () -> driveController.x().getAsBoolean(),
                                () -> true));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        driveController.leftBumper().and(() -> Objects.equals(autoLineUp.get(), "Old Line Up"))
                .whileTrue(
                        oldJoystickApproach(
                                swerveDrivetrain,
                                driveController::getLeftY,
                                driveController::getLeftX,
                                elevator::getElevatorPosition,
                                () -> driveController.x().getAsBoolean(),
                                () -> false));


//
        manipulatorController.povUp().onTrue(Commands.runOnce(elevator::nextState));
        manipulatorController.povDown().onTrue(Commands.runOnce(elevator::previousState));

//        manipulatorController.leftBumper().and(manipulatorController.rightBumper().negate())
//                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
//        manipulatorController.rightBumper().and(manipulatorController.leftBumper().negate())
//                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
//        manipulatorController.leftBumper().and(manipulatorController.rightBumper())
//                .onTrue(Commands.runOnce(() -> climber.setSpeed(1.0)));
//        manipulatorController.leftBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));
//        manipulatorController.rightBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));

        manipulatorController.leftTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_IN_SPEED)));
        manipulatorController.leftTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        manipulatorController.rightTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        manipulatorController.y().onTrue(Commands.runOnce(elevator::toggleHigh));

//        manipulatorController.a().onTrue(Commands.runOnce(pickup::lowerPickup));
//        manipulatorController.a().onFalse(Commands.runOnce(pickup::raisePickup));

//        manipulatorController.b().onTrue(Commands.runOnce(pickup::runIntakeOut));
//        manipulatorController.b().onFalse(Commands.runOnce(pickup::stopIntake));

//        manipulatorController.x().onTrue(Commands.runOnce(pickup::togglePickup));

//        driveController.a().onTrue(Commands.runOnce(() -> pickup.raisePickupToPosition(2.0)));

//        manipulatorController.povLeft().onTrue(Commands.runOnce(pickup::middlePickup));

//        elevatorButtons.button(1).onTrue(Commands.runOnce(() -> elevator.transitionToState(4)));
//        elevatorButtons.button(2).onTrue(Commands.runOnce(() -> elevator.transitionToState(3)));
//        elevatorButtons.button(3).onTrue(Commands.runOnce(() -> elevator.transitionToState(2)));
//        elevatorButtons.button(4).onTrue(Commands.runOnce(() -> elevator.transitionToState(1)));
//        elevatorButtons.button(5).onTrue(Commands.runOnce(() -> elevator.transitionToState(0)));


        driveController.rightTrigger().onTrue(Commands.runOnce(
                () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)).andThen(
                Commands.waitSeconds(0.5).andThen(
                        () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))));

    }

    private void configureControllerBindingsParade() {
        // Default command, normal field-relative drive
        swerveDrivetrain.setDefaultCommand(joystickDrive());
        driveController.a().onTrue(
                Commands.runOnce(() -> swerveDrivetrain.setPose(new Pose2d())));

        driveController.povUp().onTrue(Commands.runOnce(elevator::nextState));
        driveController.povDown().onTrue(Commands.runOnce(elevator::previousState));

        driveController.leftBumper().and(driveController.rightBumper().negate())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        driveController.rightBumper().and(driveController.leftBumper().negate())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        driveController.leftBumper().and(driveController.rightBumper())
                .onTrue(Commands.runOnce(() -> climber.setSpeed(1.0)));
        driveController.leftBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));
        driveController.rightBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));

        driveController.leftTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_IN_SPEED)));
        driveController.leftTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        driveController.rightTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_OUT_SPEED)));
        driveController.rightTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        driveController.y().onTrue(Commands.runOnce(pickup::runIntakeIn));
        driveController.y().onFalse(Commands.runOnce(pickup::stopIntake));

        driveController.b().onTrue(Commands.runOnce(pickup::runIntakeOut));
        driveController.b().onFalse(Commands.runOnce(pickup::stopIntake));

        driveController.x().onTrue(Commands.runOnce(pickup::lowerPickup));
        driveController.x().onFalse(Commands.runOnce(pickup::raisePickup));
    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("raiseElevatorToAlgae", new InstantCommand(() -> elevator.elevatorPID.setGoal(65)));
        NamedCommands.registerCommand("raiseElevatorTo4", new InstantCommand(() -> elevator.transitionToState(4)));
        NamedCommands.registerCommand("raiseElevatorTo3", new InstantCommand(() -> elevator.transitionToState(3)));
        NamedCommands.registerCommand("raiseElevatorTo2", new InstantCommand(() -> elevator.transitionToState(2)));
        NamedCommands.registerCommand("raiseElevatorTo1", new InstantCommand(() -> elevator.transitionToState(1)));
        NamedCommands.registerCommand("lowerElevator", new InstantCommand(() -> elevator.transitionToState(0)));
        NamedCommands.registerCommand("deliver", new InstantCommand(() -> delivery.setDeliveryMotor(-0.25)));
        NamedCommands.registerCommand("deliveryIn", new InstantCommand(() -> delivery.setDeliveryMotor(0.4)));
        NamedCommands.registerCommand("stopDelivery", new InstantCommand(() -> delivery.setDeliveryMotor(0)));
        NamedCommands.registerCommand("algaeOut", new InstantCommand(pickup::lowerPickup));
        NamedCommands.registerCommand("algaeIn", new InstantCommand(pickup::raisePickup));
        NamedCommands.registerCommand("algaeInFarther", new InstantCommand(() -> pickup.raisePickupToPosition(2.0)));
        NamedCommands.registerCommand("algaeHalfway", new InstantCommand(pickup::middlePickup));
        NamedCommands.registerCommand("algaeInAllTheWay", new InstantCommand(pickup::foldPickup));
        NamedCommands.registerCommand("algaeIntake", new InstantCommand(() -> pickup.setIntakeMotors(-1)));
        NamedCommands.registerCommand("algaeOuttake", new InstantCommand(() -> pickup.setIntakeMotors(1)));
        NamedCommands.registerCommand("algaeStop", new InstantCommand(() -> pickup.setIntakeMotors(0)));
        NamedCommands.registerCommand("waitForElevator", elevator.waitForElevator().withTimeout(4.0));
        NamedCommands.registerCommand("waitForElevatorPrecise", elevator.waitForElevatorPrecise().withTimeout(4.0));
        // NamedCommands.registerCommand("wait", new InstantCommand(() -> TimeUnit.wait(5000)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void autonomousInit() {
        delivery.setDefaultCommand(Commands.idle(delivery));
        vision.visionEnabled = true;
        swerveDrivetrain.visionMode = Drive.VisionMode.SIMPLE;
        visionModeString = "Anchoring";
    }
}
