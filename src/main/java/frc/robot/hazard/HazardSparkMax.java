package frc.robot.hazard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class HazardSparkMax {
    private SparkMax controller;
    private boolean useExternalEncoder;
    private RelativeEncoder integratedEncoder;
    private SparkAbsoluteEncoder externalEncoder;

    private SparkClosedLoopController closedLoop;
    private double setpoint = 0;

    private DoublePublisher velocity;
    private DoublePublisher position;
    private DoublePublisher setpointPub;
    private DoublePublisher voltage;
    private DoublePublisher current;

    public HazardSparkMax(int CAN_ID, MotorType type, SparkMaxConfig config, boolean useExternalEncoder, boolean publishToMotorTable, NetworkTable table) {
        this.useExternalEncoder = useExternalEncoder;
        controller = new SparkMax(CAN_ID, MotorType.kBrushless);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        integratedEncoder = controller.getEncoder();
        if (useExternalEncoder) {
            externalEncoder = controller.getAbsoluteEncoder();
        }

        if (publishToMotorTable) {
            velocity = table.getDoubleTopic("Velocity Rot/s").publish();
            position = table.getDoubleTopic("Position Rot").publish();
            setpointPub = table.getDoubleTopic("Control setpoint (Rot, Rot/s, Volts)").publish();
            voltage = table.getDoubleTopic("Voltage Volts").publish();
            current = table.getDoubleTopic("Current Amps").publish();
        }
    }

    public HazardSparkMax(int CAN_ID, MotorType type, SparkMaxConfig config, boolean publishToMotorTable, String name) {
        this(CAN_ID, type, config, false, publishToMotorTable, NetworkTableInstance.getDefault().getTable("SparkMax " + CAN_ID + " - " + name));
    }

    public void publishToNetworkTables() {
        if (useExternalEncoder) {
            velocity.set(getAbsVelocity());
            position.set(getAbsPosition());
        } else {
            velocity.set(getVelocity());
            position.set(getPosition());
        }
        setpointPub.set(setpoint);
        voltage.set(controller.getBusVoltage());
        current.set(controller.getOutputCurrent());
    }

    public void logEncoderState() {
        if (useExternalEncoder) {
            Constants.log(externalEncoder.getPosition());
            Constants.log(externalEncoder.getVelocity());
        } else {
            Constants.log(integratedEncoder.getPosition());
            Constants.log(integratedEncoder.getVelocity());
        }
    }

    public void set(double speed) {
        controller.set(speed);
    }

    public void setControl(double value, ControlType type) {
        closedLoop.setReference(value, type);
    }

    public double getPosition() {
        return integratedEncoder.getPosition();
    }

    public double getVelocity() {
        return integratedEncoder.getVelocity() / 60d; 
    }

    public double getAbsPosition() {
        return externalEncoder.getPosition();
    }

    public double getAbsVelocity() {
        return externalEncoder.getVelocity() / 60d;
    }
}
