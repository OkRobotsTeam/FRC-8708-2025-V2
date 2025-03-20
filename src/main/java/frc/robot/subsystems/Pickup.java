package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.Pickup.*;


public class Pickup extends SubsystemBase {
    private final SparkMax pickupMotor = new SparkMax(Constants.Pickup.Ports.DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
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
    }
    
    public Pickup() {
        rotationPID.reset();
        SparkMaxConfig pickupMotor1Config = new SparkMaxConfig();
        pickupMotor1Config.inverted(MOTOR_1_INVERTED);
        pickupMotor1Config.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        pickupMotor.configure(pickupMotor1Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        stopIntake();
    }

    public void lowerPickup() {
        rotationPID.setSetpoint(LOWERED_SETPOINT);

    }

    public void raisePickup() {
        rotationPID.setSetpoint(RAISED_SETPOINT);
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
        pickupMotor.set(power);
    }

    public BooleanSupplier isNotExtended() {
        return () -> (rotationPID.getSetpoint() != RAISED_SETPOINT);
    }

}
