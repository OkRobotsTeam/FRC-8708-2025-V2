// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Delivery.CONVEYOR_IN_SPEED;
import static frc.robot.Constants.Delivery.CONVEYOR_OUT_SPEED;
import static frc.robot.subsystems.Vision.VisionConstants.*;
import static java.lang.Math.abs;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Vision.Vision;
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


    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // AK-enabled Subsystems
    public final Drive swerveDrivetrain;

    private final Delivery delivery = new Delivery();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Pickup pickup = new Pickup();

    public final Vision vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveDrivetrain =
                new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));


        vision = new Vision(swerveDrivetrain, new VisionIOPhotonVision(alignmentCameraName, robotToAlignmentCamera));

        // Logic Triggers
        registerNamedCommands();

        // Set up auto routines
        m_autoChooser =
                new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        m_autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(swerveDrivetrain));
        m_autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerveDrivetrain));
        m_autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                swerveDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                swerveDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", swerveDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", swerveDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void init() {
        elevator.transitionToState(0);
        delivery.setDeliveryMotor(0);
        pickup.raisePickup();
        swerveDrivetrain.setSpeed(1.0);
        vision.visionEnabled = true;
    }

    public void teleopInit() {
        delivery.setDefaultCommand(Commands.run(()-> {
            delivery.setDeliveryMotor(
                    (manipulatorController.getHID().getRightTriggerAxis() / 2.0)
            );
        }, delivery));
    }

    public void teleopPeriodic() {
        double manualAdjustAmount = manipulatorController.getLeftY();
        if (abs(manualAdjustAmount) < 0.2) {
            manualAdjustAmount = 0;
        }


        elevator.manualAdjust(manualAdjustAmount * 6);
    }

    public void testInit(){
        teleopInit();
        swerveDrivetrain.setSpeed(0.1);
    }

    public void testPeriodic() {
        teleopPeriodic();
    }

    private Command joystickDrive() {
        return DriveCommands.joystickDrive(
                swerveDrivetrain,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> -driveController.getLeftTriggerAxis()
        );
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose) {
        return DriveCommands.joystickApproach(
                swerveDrivetrain,
                () -> -driveController.getLeftY(),
                approachPose);
    }

    /**
     * Button and Command mappings
     */
    private void configureControllerBindings() {
        // Default command, normal field-relative drive
        swerveDrivetrain.setDefaultCommand(joystickDrive());
        driveController.a().onTrue(
                Commands.runOnce(() -> swerveDrivetrain.setPose(new Pose2d())));
//
        manipulatorController.povUp().onTrue(Commands.runOnce(elevator::nextState));
        manipulatorController.povDown().onTrue(Commands.runOnce(elevator::previousState));

        driveController.leftBumper().whileTrue(
                joystickApproach(swerveDrivetrain::getTranslationToNearestTag)
        );

        manipulatorController.leftBumper().and(manipulatorController.rightBumper().negate()).onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.rightBumper().and(manipulatorController.leftBumper().negate()).onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.leftBumper().and(manipulatorController.rightBumper()).onTrue(Commands.runOnce(() -> climber.setSpeed(1.0)));
        manipulatorController.leftBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));
        manipulatorController.rightBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));

//        manipulatorController.leftTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_IN_SPEED)));
//        manipulatorController.leftTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));



//        Command printTriggerCommand = Commands.runOnce(() -> Debug.println("LT: ", manipulatorController.getHID().getLeftTriggerAxis()-0.5));
//
//        manipulatorController.leftTrigger().whileTrue(new RepeatCommand(printTriggerCommand));
//
//        manipulatorController.rightTrigger().whileTrue(Commands.run( ()-> {
//            Debug.println("Trigger: ", manipulatorController.getHID().getRightTriggerAxis());
//            delivery.setDeliveryMotor(
//                    (manipulatorController.getHID().getLeftTriggerAxis()-0.5)
//            );
//        }));

        manipulatorController.rightTrigger().onFalse(Commands.runOnce(() -> delivery.setDeliveryMotor(0)));

        manipulatorController.y().onTrue(Commands.runOnce(elevator::toggleHigh));

        manipulatorController.a().onTrue(Commands.runOnce(pickup::lowerPickup));
        manipulatorController.a().onFalse(Commands.runOnce(pickup::raisePickup));

        manipulatorController.b().onTrue(Commands.runOnce(pickup::runIntakeOut));
        manipulatorController.b().onFalse(Commands.runOnce(pickup::stopIntake));

        manipulatorController.x().onTrue(Commands.runOnce(pickup::runIntakeIn));
        manipulatorController.x().onFalse(Commands.runOnce(pickup::stopIntake));

        manipulatorController.rightTrigger().onTrue(Commands.runOnce(() -> delivery.setDeliveryMotor(CONVEYOR_OUT_SPEED)).andThen(() -> System.out.println("POSITION: " + swerveDrivetrain.getPose() + " ELEVATOR STATE: " + elevator.getElevatorPosition())));

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
        NamedCommands.registerCommand("deliver", new InstantCommand(() -> delivery.setDeliveryMotor(-0.4)));
        NamedCommands.registerCommand("stopDelivery", new InstantCommand(() -> delivery.setDeliveryMotor(0)));
        NamedCommands.registerCommand("algaeOut", new InstantCommand(pickup::lowerPickup));
        NamedCommands.registerCommand("algaeIn", new InstantCommand(pickup::raisePickup));
        NamedCommands.registerCommand("algaeIntake", new InstantCommand(() -> pickup.setIntakeMotors(-1)));
        NamedCommands.registerCommand("algaeOuttake", new InstantCommand(() -> pickup.setIntakeMotors(1)));
        NamedCommands.registerCommand("algaeStop", new InstantCommand(() -> pickup.setIntakeMotors(0)));
        NamedCommands.registerCommand("waitForElevator", elevator.waitForElevator().withTimeout(4.0));
        NamedCommands.registerCommand("waitForElevatorPrecise", elevator.waitForElevatorPrecise().withTimeout(4.0));
//        NamedCommands.registerCommand("wait",  new InstantCommand(() -> TimeUnit.wait(5000)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.get();
    }

    public void autonomousInit() {
        delivery.setDefaultCommand(Commands.idle(delivery));
        vision.visionEnabled = true;
    }
}
