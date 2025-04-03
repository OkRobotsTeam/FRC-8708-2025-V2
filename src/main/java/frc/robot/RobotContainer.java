// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Delivery.CONVEYOR_IN_SPEED;
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


    // AK-enabled Subsystems
    public final Drive swerveDrivetrain;

    private final Delivery delivery = new Delivery();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Pickup pickup = new Pickup();

    public final Vision vision;


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


        autoLineUp = new LoggedDashboardChooser<>("Drive Speed");

        autoLineUp.addDefaultOption("Old Line Up", "Old Line Up");
        autoLineUp.addOption("New Line Up", "New Line Up");
        // Set up auto routines
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
        }, delivery));
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
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
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

        manipulatorController.y().onTrue(Commands.runOnce(elevator::toggleHigh));

        manipulatorController.a().onTrue(Commands.runOnce(pickup::lowerPickup));
        manipulatorController.a().onFalse(Commands.runOnce(pickup::raisePickup));

        manipulatorController.b().onTrue(Commands.runOnce(pickup::runIntakeOut));
        manipulatorController.b().onFalse(Commands.runOnce(pickup::stopIntake));

        manipulatorController.x().onTrue(Commands.runOnce(pickup::togglePickup));

        manipulatorController.povLeft().onTrue(Commands.runOnce(pickup::middlePickup));

        elevatorButtons.button(1).onTrue(Commands.runOnce(() -> elevator.transitionToState(4)));
        elevatorButtons.button(2).onTrue(Commands.runOnce(() -> elevator.transitionToState(3)));
        elevatorButtons.button(3).onTrue(Commands.runOnce(() -> elevator.transitionToState(2)));
        elevatorButtons.button(4).onTrue(Commands.runOnce(() -> elevator.transitionToState(1)));
        elevatorButtons.button(5).onTrue(Commands.runOnce(() -> elevator.transitionToState(0)));


        driveController.rightTrigger().onTrue(Commands.runOnce(
                () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)).andThen(
                        Commands.waitSeconds(0.5).andThen(
                                () -> manipulatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))));

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
        NamedCommands.registerCommand("deliver", new InstantCommand(() -> delivery.setDeliveryMotor(-0.3)));
        NamedCommands.registerCommand("deliveryIn", new InstantCommand(() -> delivery.setDeliveryMotor(0.4)));
        NamedCommands.registerCommand("stopDelivery", new InstantCommand(() -> delivery.setDeliveryMotor(0)));
        NamedCommands.registerCommand("algaeOut", new InstantCommand(pickup::lowerPickup));
        NamedCommands.registerCommand("algaeIn", new InstantCommand(pickup::raisePickup));
        NamedCommands.registerCommand("algaeHalfway", new InstantCommand(() -> pickup.movePickupToPosition(5)));
        NamedCommands.registerCommand("algaeInAllTheWay", new InstantCommand(() -> pickup.movePickupToPosition(0)));
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
    }
}
