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
    private int CAN_ID;
    private SparkMax controller;
    private SparkMaxConfig config;
    private boolean useExternalEncoder;
    private RelativeEncoder integratedEncoder;
    private SparkAbsoluteEncoder externalEncoder;

    private SparkClosedLoopController closedLoop;
    private double setpoint = 0;

    private boolean publishToMotorTable;
    private DoublePublisher velocity;
    private DoublePublisher position;
    private DoublePublisher setpointPub;
    private DoublePublisher voltage;
    private DoublePublisher current;

    public HazardSparkMax(int CAN_ID, MotorType type, SparkMaxConfig config, boolean useExternalEncoder, boolean publishToMotorTable, NetworkTable table) {
        this.CAN_ID = CAN_ID;
        this.config = config;
        this.useExternalEncoder = useExternalEncoder;
        this.publishToMotorTable = publishToMotorTable;
        controller = new SparkMax(CAN_ID, MotorType.kBrushless);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        integratedEncoder = controller.getEncoder();
        if (useExternalEncoder) {
            externalEncoder = controller.getAbsoluteEncoder();
        }

        closedLoop = controller.getClosedLoopController();

        if (publishToMotorTable) {
            velocity = table.getDoubleTopic("Velocity RPS").publish();
            position = table.getDoubleTopic("Position Rotations").publish();
            setpointPub = table.getDoubleTopic("Control setpoint (Rotations, RPS, Volts)").publish();
            voltage = table.getDoubleTopic("Voltage Volts").publish();
            current = table.getDoubleTopic("Current Amps").publish();
        }
    }

    public HazardSparkMax(int CAN_ID, MotorType type, SparkMaxConfig config, boolean publishToMotorTable, String name) {
        this(CAN_ID, type, config, false, publishToMotorTable, NetworkTableInstance.getDefault().getTable("SparkMax " + CAN_ID + " - " + name));
    }

    public void configurePID(double p, double i, double d) {
      config.closedLoop.pid(p, i, d);
      controller.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /***
     * Publishes motor information to the network table
     */
    public void publishToNetworkTables() {
        if (!publishToMotorTable) return;
        if (useExternalEncoder) {
            velocity.set(getAbsVelocity());
            position.set(getAbsPosition());
        } else {
            velocity.set(getVelocity());
            position.set(getPosition());
        }
        setpointPub.set(setpoint);
        voltage.set(controller.getBusVoltage());
        current.set(controller.getAppliedOutput());
    }

    /***
     * Logs encoder information to the roborio log
     */
    public void logEncoderState() {
        if (useExternalEncoder) {
            Constants.log(CAN_ID + " Position: " + externalEncoder.getPosition());
            Constants.log(CAN_ID + " Velocity: " + externalEncoder.getVelocity());
        } else {
            Constants.log(CAN_ID + " Position: " + integratedEncoder.getPosition());
            Constants.log(CAN_ID + " Velocity: " + integratedEncoder.getVelocity());
        }
    }

    /***
     * Sets the duty cycle of the controller. Calls SparkMax.set();
     * @param speed
     */
    public void set(double speed) {
        controller.set(speed);
    }

    /***
     * Sets the controller reference value based on the selected control mode. Calls SparkClosedLoopController.setReference();
     * @param value
     * @param type
     */
    public void setControl(double value, ControlType type) {
        setpoint = value;
        closedLoop.setReference(value, type);
    }

    /***
     * Sets the controller setpoint value, but does not send it to the motor controller. Use to publish a value to networktables using the motor's setpoint key.
     * @param value
     */
    public void setSetpointPublishingOnly(double value) {
        setpoint = value;
    }

    /***
     * Sets the encoder value
     * @param value
     */
    public void setEncoder(double value) {
        integratedEncoder.setPosition(value);
    }

    /***
     * Returns the position of the encoder in rotations
     * @return
     */
    public double getPosition() {
        return integratedEncoder.getPosition();
    }

    /***
     * Returns the position of the encoder in radians
     * @return
     */
    public double getPositionRadians() {
        return integratedEncoder.getPosition() * 2 * Math.PI;
    }

    /***
     * Returns the velocity of the encoder in rotations per second
     * @return
     */
    public double getVelocity() {
        return integratedEncoder.getVelocity() / 60d; 
    }

    /***
     * Returns the velocity of the encoder in radians per second
     * @return
     */
    public double getVelocityRadPS() {
        return integratedEncoder.getVelocity() * 2 * Math.PI;
    }

    /***
     * Returns the position of the absolute encoder in rotations
     * @return
     */
    public double getAbsPosition() {
        return externalEncoder.getPosition();
    }

    /***
     * Returns the absolute velocity of the encoder in rotations per second
     * @return
     */
    public double getAbsVelocity() {
        return externalEncoder.getVelocity() / 60d;
    }
}
