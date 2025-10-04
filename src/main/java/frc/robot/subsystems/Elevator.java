package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Debug;
import frc.robot.MathUtils;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(Ports.MOTOR_1);
    private final TalonFX motor2 = new TalonFX(Ports.MOTOR_2);
    Encoder encoder = new Encoder(Ports.ENCODER_CHANNEL_A, Ports.ENCODER_CHANNEL_B, ENCODER_REVERSED, ENCODER_ENCODING_TYPE);

    public final ProfiledPIDController elevatorPID = new ProfiledPIDController(KP, KI, KD, ELEVATOR_TRAPEZOID_PROFILE);
    public int currentState = 0;
    private double manualAdjustAmount = 0.0;
    private double lastTime = 0;

    public Elevator() {
        stop();
        encoder.reset();
        // Configures the encoder to return a distance of 1 for every 8192 pulses (one revolution of the REV Through-bore)
        // Also changes the units of getRate
        encoder.setDistancePerPulse(20.0 / 2462.0);
        elevatorPID.reset(getElevatorPosition());
        transitionToState(currentState);
    }

    @Override
    public void periodic() {
        double pidOutput = elevatorPID.calculate(getElevatorPosition());

        if (currentState == 0 && Math.abs(getElevatorPosition()) < 2.0) {
            setMotors(pidOutput);
            Logger.recordOutput("Elevator/MotorPower", pidOutput);
        } else {
            setMotors(pidOutput + KG);
            Logger.recordOutput("Elevator/MotorPower", pidOutput+KG);

        }
    }

    public double getElevatorPosition() {
        return encoder.getDistance();
    }

    public void transitionToState(int state) {
        try {
            manualAdjustAmount = 0;
            currentState = state;
            System.out.println("Moving To State: " + currentState);
            updateManualAdjust();
        } catch (Exception e) {
            System.out.println("Error setting elevator to state: " + e);
        }
    }

    public void updateManualAdjust() {
        try {
            elevatorPID.setGoal(MathUtils.clamp(States[currentState] + manualAdjustAmount, 0, 65));
        } catch (Exception e) {
            System.out.println("Current State out of range: " + e);
        }
    }

    public void stop() {
        setMotors(0.0);
    }

    public void nextState() {
        if (currentState < States.length - 1) {
            transitionToState(currentState + 1);
        }
    }

    public void previousState() {
        if (currentState > 0) {
            transitionToState(currentState - 1);
        }
    }

    public void setMotors(double power) {
        if (getElevatorPosition() > 12.0) {
            power = MathUtil.clamp(power, -0.15, 1);
        } else {
            power = MathUtil.clamp(power, 0, 1);
        }
        motor1.set(power);
        motor2.set(power);
    }

    double getTime() {
        return System.currentTimeMillis() / 1000.0;
    }

    public void manualAdjust(double amount) {
        double currentTime = getTime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        manualAdjustAmount = manualAdjustAmount - amount * deltaTime;
        updateManualAdjust();
    }

    public void manualAdjustOut() {
        manualAdjustAmount += MANUAL_ADJUST_STEP;
        updateManualAdjust();
    }

    public void ManualAdjustIn() {
        manualAdjustAmount -= MANUAL_ADJUST_STEP;
        updateManualAdjust();
    }

    public Command waitForElevator() {
        return Commands.waitUntil(() -> {
            // Debug.println("Checking Elevator: P: ", getElevatorPosition(), " SP:", elevatorPID.getGoal().position, " Diff: ",
            // Math.abs(getElevatorPosition() - elevatorPID.getGoal().position));
            return Math.abs(getElevatorPosition() - elevatorPID.getGoal().position) < 5.0;
        });
    }

    public Command waitForElevatorPrecise() {
        return Commands.waitUntil(() -> {
            // Debug.println("Checking Elevator: P: ", getElevatorPosition(), " SP:", elevatorPID.getGoal().position, " Diff: ",
            // Math.abs(getElevatorPosition() - elevatorPID.getGoal().position));
            return Math.abs(getElevatorPosition() - elevatorPID.getGoal().position) < 1.0;
        });
    }

    public void toggleHigh() {
        if (currentState == 4) {
            transitionToState(0);
        } else {
            transitionToState(4);
        }
    }
}
