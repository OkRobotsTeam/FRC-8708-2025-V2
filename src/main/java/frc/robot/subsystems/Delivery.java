package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Delivery.*;


public class Delivery extends SubsystemBase {
    private final TalonFX deliveryMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID);


    @SuppressWarnings("removal")
    public Delivery() {
        SparkMaxConfig deliveryMotorConfig = new SparkMaxConfig();
        deliveryMotorConfig.inverted(DELIVERY_MOTOR_INVERTED);
        deliveryMotorConfig.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        deliveryMotor.setInverted(CONVEYOR_MOTOR_INVERTED);
        stopDelivery();
    }


    public void stopDelivery() {
        setDeliveryMotor(0.0);
    }

    public void setDeliveryMotor(double power) {
        deliveryMotor.set(power);
    }


}
