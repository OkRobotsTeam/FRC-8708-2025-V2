package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Delivery.*;


public class Delivery extends SubsystemBase {
    private final SparkMax deliveryMotor = new SparkMax(Ports.DELIVERY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final TalonFX conveyorMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID);


    @SuppressWarnings("removal")
    public Delivery() {
        SparkMaxConfig deliveryMotorConfig = new SparkMaxConfig();
        deliveryMotorConfig.inverted(DELIVERY_MOTOR_INVERTED);
        deliveryMotorConfig.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        deliveryMotor.configure(deliveryMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        conveyorMotor.setInverted(CONVEYOR_MOTOR_INVERTED);
        stopDelivery();
    }

    public Command runDeliveryIn() {
        return runOnce(() -> {
            setDeliveryMotor(DELIVERY_IN_SPEED);
        });
    }

    public Command runDeliveryOut() {
        return runOnce(() -> {
            setDeliveryMotor(DELIVERY_OUT_SPEED);
        });
    }

    public Command stopDeliveryCmd() {
        return runOnce(() -> {
            setDeliveryMotor(0);
        });
    }

    public void stopDelivery() {
        setDeliveryMotor(0.0);
    }

    public void setDeliveryMotor(double power) {
        deliveryMotor.set(power);
    }



    public Command runConveyorIn() {
        return runOnce(() -> {
            setConveyorMotor(CONVEYOR_IN_SPEED);
        });
    }

    public Command runConveyorOut() {
        return runOnce(() -> {
            setConveyorMotor(CONVEYOR_OUT_SPEED);
        });
        
    }

    public Command stopConveyorCmd() {
        return runOnce(() -> {
            setConveyorMotor(0);
        });
    }

    public void stopConveyor() {
        setConveyorMotor(0.0);
    }

    public void setConveyorMotor(double power) {
        conveyorMotor.set(power);
    }


}
