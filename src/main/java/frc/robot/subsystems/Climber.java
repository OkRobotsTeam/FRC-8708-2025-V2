package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Climber.*;


public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(Constants.Climber.Ports.CLIMBER_MOTOR);
    private final PIDController climberPID = new PIDController(KP, KI, KD);

    public Climber() {
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        climberPID.reset();
    }

    /**
     * Updates the field relative position of the robot.
     * Called automatically by command scheduler
     */
    @Override
    public void periodic() {
        double pidOutput = climberPID.calculate(climberMotor.getPosition().getValueAsDouble());
//        climberMotor.set(pidOutput);
    }

        public void setSpeed(double speed) {
        climberMotor.set(speed);
        }

//    public void climb() {
//        climberPID.setSetpoint(CLIMBING_SETPOINT);
//    }
//    public void dock() {
//        climberPID.setSetpoint(DOCKING_SETPOINT);
//    }
}
