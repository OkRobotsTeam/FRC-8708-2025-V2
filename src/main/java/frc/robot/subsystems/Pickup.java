package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.Pickup.*;

public class Pickup extends SubsystemBase {
    // private final SparkMax pickupMotor = new SparkMax(Constants.Pickup.Ports.DELIVERY_MOTOR_ID,
    // SparkLowLevel.MotorType.kBrushless);
    private final TalonFX pickupMotorKraken = new TalonFX(Ports.DELIVERY_MOTOR_ID);
    private final TalonFX rotationMotor = new TalonFX(Ports.ROTATION_MOTOR);
    public final PIDController rotationPID = new PIDController(KP, KI, KD);

    /**
     * Updates the field relative position of the robot.
     * Called automatically by command scheduler
     */
    @Override
    public void periodic() {
        double pidOutput = rotationPID.calculate(rotationMotor.getPosition().getValueAsDouble());
        rotationMotor.set(pidOutput);
        Logger.recordOutput("Pickup/Setpoint", rotationPID.getSetpoint());

    }

    public Pickup() {
        rotationPID.reset();
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        pickupMotorKraken.setInverted(MOTOR_1_INVERTED);
        stopIntake();
    }

    public void lowerPickup() {
        rotationPID.setSetpoint(LOWERED_SETPOINT);
    }
    public void foldPickup() {
        rotationPID.setSetpoint(0.0);
    }

    public void raisePickup() {
        rotationPID.setSetpoint(RAISED_SETPOINT);
    }

    public void raisePickupToPosition(Double position) {
        rotationPID.setSetpoint(position);
    }


    public void middlePickup() {
        rotationPID.setSetpoint(MIDDLE_SETPOINT);
    }

    public void movePickupToPosition(double position) {
        rotationPID.setSetpoint(position);
    }

    public void runIntakeIn() {
        setIntakeMotors(INTAKE_IN_SPEED);
    }

    public void runIntakeOut() {
        setIntakeMotors(INTAKE_OUT_SPEED);
    }

    public void stopIntake() {
        setIntakeMotors(0.0);
    }

    public void setIntakeMotors(double power) {
        // pickupMotor.set(power);
        pickupMotorKraken.set(power);
    }

    public void manualAdjust(double manualAdjustAmount) {
        rotationPID.setSetpoint(rotationPID.getSetpoint() + manualAdjustAmount);
    }

    public BooleanSupplier isNotExtended() {
        return () -> (rotationPID.getSetpoint() != RAISED_SETPOINT);
    }

    public void togglePickup() {
        if (Math.abs(pickupMotorKraken.get()) > 0) {
            stopIntake();
        } else runIntakeIn();
    }

}
