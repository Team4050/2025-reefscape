package frc.robot.hazard;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class HazardSparkMax {
  private int CAN_ID;
  private SparkMax controller;
  private SparkMaxConfig config;
  private double currentLimit;
  private boolean useExternalEncoder;
  private RelativeEncoder integratedEncoder;
  private SparkAbsoluteEncoder externalEncoder;

  private SparkClosedLoopController closedLoop;
  private double setpoint = 0;

  private double lastVelocityRotations;
  private boolean publishToMotorTable;
  private StringPublisher controlMode;
  private DoublePublisher dutyCycle;
  private DoublePublisher velocity;
  private DoublePublisher position;
  private DoublePublisher rotations;
  private DoublePublisher absolutePosition;
  private DoublePublisher setpointPub;
  private DoublePublisher setpoint2Pub;
  private DoublePublisher voltage;
  private DoublePublisher current;

  private Trigger loadTrigger;

  public HazardSparkMax(
      int CAN_ID,
      MotorType type,
      double currentLimit,
      SparkMaxConfig config,
      boolean useExternalEncoder,
      boolean publishToMotorTable,
      NetworkTable table) {
    this.CAN_ID = CAN_ID;
    this.config = config;
    this.useExternalEncoder = useExternalEncoder;
    this.publishToMotorTable = publishToMotorTable;
    this.currentLimit = currentLimit;
    loadTrigger = new Trigger(this::isUnderLoad);
    controller = new SparkMax(CAN_ID, MotorType.kBrushless);
    controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    integratedEncoder = controller.getEncoder();
    Constants.log(CAN_ID + " is a follower? " + controller.isFollower());
    if (useExternalEncoder) {
      externalEncoder = controller.getAbsoluteEncoder();
      Constants.log(CAN_ID + " Absolute encoder starting at " + externalEncoder.getPosition());
    }

    closedLoop = controller.getClosedLoopController();

    if (publishToMotorTable) {
      velocity = table.getDoubleTopic("Velocity RadPS").publish();
      position = table.getDoubleTopic("Position Radians").publish();
      rotations = table.getDoubleTopic("Position Rotations").publish();
      absolutePosition = table.getDoubleTopic("Absotlute Position Radians").publish();
      setpointPub = table.getDoubleTopic("Control setpoint (Rotations, RPS, Volts)").publish();
      setpoint2Pub = table.getDoubleTopic("Alternate control setpoint").publish();
      voltage = table.getDoubleTopic("Stator Voltage").publish();
      dutyCycle = table.getDoubleTopic("Motor Duty Cycle").publish();
      current = table.getDoubleTopic("Output Current").publish();
      controlMode = table.getStringTopic("Control Mode").publish();
    }
  }

  public HazardSparkMax(
      int CAN_ID, MotorType type, double currentLimit, SparkMaxConfig config, boolean publishToMotorTable, String name) {
    this(
        CAN_ID,
        type,
        currentLimit,
        config,
        false,
        publishToMotorTable,
        NetworkTableInstance.getDefault().getTable("SparkMax " + CAN_ID + " - " + name));
  }

  public HazardSparkMax(
      int CAN_ID,
      MotorType type,
      double currentLimit,
      SparkMaxConfig config,
      boolean useExternalEncoder,
      boolean publishToMotorTable,
      String name) {
    this(
        CAN_ID,
        type,
        currentLimit,
        config,
        useExternalEncoder,
        publishToMotorTable,
        NetworkTableInstance.getDefault().getTable("SparkMax " + CAN_ID + " - " + name));
  }

  /***
   * Reconfigures the spark max with the desired control values
   * @param p
   * @param i
   * @param d
   */
  public void configurePID(double p, double i, double d) {
    config.closedLoop.pid(p, i, d);
    controller.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      config.idleMode(IdleMode.kBrake);
    } else {
      config.idleMode(IdleMode.kCoast);
    }
  }

  /***
   * Publishes motor information to the network table
   */
  public void publishToNetworkTables() {
    if (!publishToMotorTable) return;
    velocity.set(getVelocityRadPS());
    position.set(getPositionRadians());
    rotations.set(getPosition());
    if (useExternalEncoder) {
      absolutePosition.set(getAbsPositionRadians());
    }
    setpointPub.set(setpoint);
    dutyCycle.set(controller.get());
    voltage.set(controller.getAppliedOutput() * controller.getBusVoltage());
    current.set(controller.getOutputCurrent());
  }

  /***
   * Logs encoder information to the roborio log
   */
  public void logEncoderState() {
    if (useExternalEncoder) {
      Constants.log(CAN_ID + " Position: " + getAbsPositionRadians());
      Constants.log(CAN_ID + " Velocity: " + getAbsVelocity());
    } else {
      Constants.log(CAN_ID + " Position: " + getPositionRadians());
      Constants.log(CAN_ID + " Velocity: " + getVelocityRadPS());
    }
  }

  public boolean isUnderLoad() {
    return controller.getOutputCurrent() > currentLimit / 10;
  }

  public Voltage getVoltage() {
    return Voltage.ofBaseUnits(controller.getAppliedOutput() * controller.getBusVoltage(), Volts); //TODO: verify if this is an accurate representation of voltage passing through the motor
  }

  public Current getCurrent() {
    return Current.ofBaseUnits(controller.getOutputCurrent(), Amps);
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
    return integratedEncoder.getVelocity() * (2.0 * Math.PI) / 60d;
  }

  /***
   * Returns the position of the absolute encoder in rotations
   * @return
   */
  public double getAbsPosition() {
    return externalEncoder.getPosition() / 360.0;
  }

  public double getAbsPositionRadians() {
    double deg = externalEncoder.getPosition();
    if (deg < 0 || deg > 360) {
      Constants.log(CAN_ID + ": aBS detected, degrees = " + deg);
      return setpoint;
    }
    if (deg > 180) {
      deg -= 360;
    }
    return Math.toRadians(deg);
  }

  /***
   * Returns the absolute velocity of the encoder in rotations per second
   * @return
   */
  public double getAbsVelocity() {
    return externalEncoder.getVelocity() / 60d;
  }

  public double getAbsVelocityRadPS() {
    double deg = externalEncoder.getVelocity() / 60d;
    return Math.toRadians(deg);
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
    if (publishToMotorTable) {
      controlMode.set(type.toString());
    }
  }

  /***
   * Sets the controller setpoint value, but does not send it to the motor controller. Use to publish a value to networktables using the motor's setpoint key.
   * @param value
   */
  public void setSetpointPublishingOnly(double value) {
    setpoint2Pub.set(value);
  }

  /***
   * Sets the encoder value in rotations
   * @param value
   */
  public void setEncoder(double value) {
    Constants.log("Setting encoder rotations to " + value);
    integratedEncoder.setPosition(value);
  }

  /***
   * Sets the encoder value in radians
   * @param value
   */
  public void setEncoderRadians(double value) {
    Constants.log("Setting encoder radians to " + value);
    integratedEncoder.setPosition(value / (2 * Math.PI));
  }
}
