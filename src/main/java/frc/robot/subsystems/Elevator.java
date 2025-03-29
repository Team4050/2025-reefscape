package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.hazard.HazardArm;
import frc.robot.hazard.HazardSparkMax;

public class Elevator extends SubsystemBase {
  private Mechanism elevatorSim = new Mechanism(this::setElevatorVoltage, this::logElevator, this);
  private Mechanism shoulderSim = new Mechanism(this::setShoulderVoltage, this::logShoulder, this);
  private Mechanism wristSim = new Mechanism(this::setWristVoltage, this::logWrist, this);

  private SysIdRoutine elevatorRoutine = new SysIdRoutine(new Config(Voltage.ofBaseUnits(1, Volt).per(Second), Voltage.ofBaseUnits(0.1, Volts), Time.ofBaseUnits(10, Seconds), this::recordElevatorState), elevatorSim);
  private SysIdRoutine shoulderRoutine = new SysIdRoutine(new Config(Voltage.ofBaseUnits(1, Volt).per(Second), Voltage.ofBaseUnits(0.1, Volts), Time.ofBaseUnits(10, Seconds), this::recordShoulderState), shoulderSim);
  private SysIdRoutine wristRoutine = new SysIdRoutine(new Config(Voltage.ofBaseUnits(1, Volt).per(Second), Voltage.ofBaseUnits(0.1, Volts), Time.ofBaseUnits(10, Seconds), this::recordWristState), wristSim);

  public boolean safeMode = false;
  public boolean isScoringL4 = false;
  public boolean isLoading = false;
  public int position = 0;

  public BooleanSupplier elevatorAtLimits = this::isElevatorAtLimits;
  public BooleanSupplier shoulderAtLimits = this::isShoulderAtLimits;
  public BooleanSupplier wristAtLimits = this::isWristAtLimits;

  private HazardSparkMax leadMotor;
  private HazardSparkMax followerMotor;
  private HazardSparkMax shoulderMotor;
  private HazardSparkMax shoulderFollower;
  private HazardSparkMax wristMotor;

  // private SparkMax elevatorWrist; // Not in CAN yet
  // private SparkClosedLoopController wristMotorController;
  private PIDController elevatorPID;
  private ElevatorFeedforward elevatorFF;

  private HazardArm shoulder;
  private HazardArm wrist;

  private double[] kinematicsTarget = { 0, 0 };
  private double elevatorSetpoint = 0;
  private boolean lockElevator = false;
  private double shoulderSetpoint = 0;
  private boolean lockShoulder = false;
  private double wristSetpoint = 0;
  private boolean lockWrist = false;

  private double[] bump = { 0, 0, 0 };

  private NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
  private DoublePublisher setpointPublisher =
      elevatorTable.getDoubleTopic("Elevator setpoint").publish();
  private DoubleArrayPublisher kinematicsPublisher =
      elevatorTable.getDoubleArrayTopic("Elevator kinematics").publish();
  private DoubleArrayPublisher inverseKinematicsPublisher =
    elevatorTable.getDoubleArrayTopic("Elevator IK target").publish();
    private StringPublisher routinePublisher = elevatorTable.getStringTopic("Current routine state").publish();

  private SparkMaxConfig followConfig = new SparkMaxConfig();
  private SparkMaxConfig leadConfig = new SparkMaxConfig();
  private SparkMaxConfig shoulderConfig = new SparkMaxConfig();
  private SparkMaxConfig shoulderFollowerConfig = new SparkMaxConfig();
  private SparkMaxConfig wristConfig = new SparkMaxConfig();

  public Elevator(boolean tuningMode) {
    followConfig.smartCurrentLimit(Constants.Elevator.currentLimit);
    followConfig.idleMode(IdleMode.kBrake);
    followConfig.follow(Constants.Elevator.front, true);
    followConfig.encoder.positionConversionFactor(Constants.Elevator.gearboxReduction);
    followConfig.encoder.velocityConversionFactor(Constants.Elevator.gearboxReduction);

    leadConfig.smartCurrentLimit(Constants.Elevator.currentLimit);
    leadConfig.idleMode(IdleMode.kBrake);
    leadConfig.inverted(true);
    leadConfig.closedLoop.outputRange(-1, 1);
    leadConfig.closedLoop.pid(0.3, 0.002, 0.0001);
    leadConfig.closedLoop.iMaxAccum(10);
    leadConfig.closedLoop.iZone(0.3);
    leadConfig.closedLoop.maxMotion.allowedClosedLoopError(0.03);
    leadConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    leadConfig.closedLoop.maxMotion.maxAcceleration(500, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.maxMotion.maxVelocity(500, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leadConfig.encoder.positionConversionFactor(Constants.Elevator.gearboxReduction);
    leadConfig.encoder.velocityConversionFactor(Constants.Elevator.gearboxReduction);

    shoulderConfig.smartCurrentLimit(Constants.Shoulder.currentLimit);
    shoulderConfig.idleMode(IdleMode.kBrake);
    shoulderConfig.inverted(true); // Change to gear from belt drive
    shoulderConfig.closedLoop.pidf(0.25, 0.005, 0.05, 0.25);
    shoulderConfig.closedLoop.iMaxAccum(0.085);
    shoulderConfig.closedLoop.outputRange(-0.3, 0.3);
    shoulderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    shoulderConfig.closedLoop.maxMotion.maxAcceleration(50);
    shoulderConfig.closedLoop.maxMotion.maxVelocity(100);
    shoulderConfig.closedLoop.maxMotion.allowedClosedLoopError(0.02);
    shoulderConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    shoulderConfig.encoder.positionConversionFactor(Constants.Shoulder.gearboxReduction);
    shoulderConfig.encoder.velocityConversionFactor(Constants.Shoulder.gearboxReduction);
    shoulderConfig.absoluteEncoder.inverted(true);
    shoulderConfig.absoluteEncoder.zeroOffset(Constants.Shoulder.encoderOffset);
    shoulderConfig.absoluteEncoder.positionConversionFactor(360);

    shoulderFollowerConfig.smartCurrentLimit(Constants.Shoulder.currentLimit);
    shoulderFollowerConfig.idleMode(IdleMode.kBrake);
    shoulderFollowerConfig.inverted(false);
    shoulderFollowerConfig.follow(Constants.Shoulder.leadCAN, true); // TODO: is inverted?
    shoulderFollowerConfig.encoder.positionConversionFactor(Constants.Shoulder.gearboxReduction);
    shoulderFollowerConfig.encoder.velocityConversionFactor(Constants.Shoulder.gearboxReduction);

    wristConfig.smartCurrentLimit(Constants.Wrist.currentLimit);
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.inverted(true);
    wristConfig.closedLoop.pid(0.2, 0.00005, 0);
    wristConfig.closedLoop.iMaxAccum(5);
    wristConfig.closedLoop.outputRange(-0.4, 0.4);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    wristConfig.closedLoop.maxMotion.maxAcceleration(50);
    wristConfig.closedLoop.maxMotion.maxVelocity(100); // 1 rotation/s
    wristConfig.closedLoop.maxMotion.allowedClosedLoopError(0.01);
    wristConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    wristConfig.encoder.positionConversionFactor(Constants.Wrist.gearboxReduction);
    wristConfig.encoder.velocityConversionFactor(Constants.Wrist.gearboxReduction);

    leadMotor =
        new HazardSparkMax(
            Constants.Elevator.back, MotorType.kBrushless,
            Constants.Elevator.currentLimit, leadConfig, false, true, elevatorTable);
    followerMotor =
        new HazardSparkMax(
            Constants.Elevator.front,
            MotorType.kBrushless,
            Constants.Elevator.currentLimit,
            followConfig,
            false,
            false,
            elevatorTable);
    shoulderMotor =
        new HazardSparkMax(
            Constants.Shoulder.leadCAN, MotorType.kBrushless, Constants.Shoulder.currentLimit, shoulderConfig, true, true, "Shoulder");
    shoulderFollower = new HazardSparkMax(Constants.Shoulder.followerCAN, MotorType.kBrushless, Constants.Shoulder.currentLimit, shoulderFollowerConfig, false, true, "Shoulder follower");
    wristMotor =
        new HazardSparkMax(Constants.Wrist.CAN, MotorType.kBrushless, Constants.Wrist.currentLimit, wristConfig, true, "Wrist");

    shoulderSetpoint = Constants.Shoulder.startingRotationRadians;
    wristSetpoint = Constants.Wrist.startingRotationRadians;

    leadMotor.setEncoder(Constants.Elevator.startingExtension);
    followerMotor.setEncoder(Constants.Elevator.startingExtension);
    shoulderMotor.setEncoder(Constants.Shoulder.startingRotationRadians / (2 * Math.PI));
    shoulderFollower.setEncoder(Constants.Shoulder.startingRotationRadians / (2 * Math.PI));
    wristMotor.setEncoder(Constants.Wrist.startingRotationRadians / (2 * Math.PI));

    elevatorPID =
        new PIDController(
            0.1, 0, 0.05); // Unused, backup if manual feedforward calculation is needed
    elevatorFF = new ElevatorFeedforward(0, 0, 0, 0);

    //Constants.log("Shoulder NEO Kv: " + (49.0 / Constants.Shoulder.motor.KvRadPerSecPerVolt)); No longer needed

    shoulder =
        new HazardArm(
            shoulderMotor,
            0,
            true,
            0.1, // 0.1,
            0.55, // 1.0,
            0.3,
            1.6,
            0.08,
            0,
            10,//Math.toRadians(90), //90deg/s
            2.8, //Math.toRadians(270), //Reach max speed in 1/3 seconds TODO: try these values
            "Shoulder",
            tuningMode,
            true);
    wrist =
        new HazardArm( // Adjust Kstatic for small movements
            wristMotor,
            0,
            false,
            0.1,
            0.19,
            0.00,
            1.0,//1.8,
            0.12,
            0.15,
            2,
            2, "Wrist", tuningMode);

    shoulder.setpoint(shoulderSetpoint);
    wrist.setpoint(wristSetpoint);

    if (tuningMode) {
      SmartDashboard.putNumber("Elevator Kp", 0.35);
      SmartDashboard.putNumber("Elevator Ki", 0.00001);
      SmartDashboard.putNumber("Elevator Kd", 0);

      SmartDashboard.putData(
          "Reconfigure controllers",
          new InstantCommand(
              () -> {
                this.reconfigure();
              },
              this));
    }

    SmartDashboard.putNumber("Elevator setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Shoulder setpoint", shoulderSetpoint);
    SmartDashboard.putNumber("Wrist setpoint", wristSetpoint);
  }

  public void init() {
    shoulder.unstop();
  }

  //************************************************ Getters ************************************************//

  //**************************** State stuff ****************************//

  public Command getElevatorRoutines() {
    return Commands.sequence(
      elevatorRoutine.quasistatic(Direction.kForward).until(elevatorAtLimits),
      elevatorRoutine.quasistatic(Direction.kReverse).until(elevatorAtLimits),
      elevatorRoutine.dynamic(Direction.kForward).until(elevatorAtLimits),
      elevatorRoutine.dynamic(Direction.kReverse).until(elevatorAtLimits)
    );
  }

  public Command getShoulderRoutines() {
    return Commands.sequence(
      shoulderRoutine.quasistatic(Direction.kForward).until(shoulderAtLimits),
      shoulderRoutine.quasistatic(Direction.kReverse).until(shoulderAtLimits),
      shoulderRoutine.dynamic(Direction.kForward).until(shoulderAtLimits),
      shoulderRoutine.dynamic(Direction.kReverse).until(shoulderAtLimits)
    );
  }

  public Command getWristRoutines() {
    return Commands.sequence(
      wristRoutine.quasistatic(Direction.kForward).until(wristAtLimits),
      wristRoutine.quasistatic(Direction.kReverse).until(wristAtLimits),
      wristRoutine.dynamic(Direction.kForward).until(wristAtLimits),
      wristRoutine.dynamic(Direction.kReverse).until(wristAtLimits)
    );
  }

  public void logElevator(SysIdRoutineLog log) {
    var leadLog = log.motor("Elevator lead");
    leadLog.linearPosition(Distance.ofBaseUnits(leadMotor.getPosition() * Constants.Elevator.gearboxRotationsToHeightMM, Millimeters));
    leadLog.linearVelocity(LinearVelocity.ofBaseUnits(leadMotor.getVelocity() * Constants.Elevator.gearboxRotationsToHeightMM / 1000.0, MetersPerSecond));
    leadLog.voltage(leadMotor.getVoltage());
    leadLog.current(leadMotor.getCurrent());
    var followerLog = log.motor("Elevator follower");
    followerLog.linearPosition(Distance.ofBaseUnits(followerMotor.getPosition() * Constants.Elevator.gearboxRotationsToHeightMM, Millimeters));
    followerLog.linearVelocity(LinearVelocity.ofBaseUnits(followerMotor.getVelocity() * Constants.Elevator.gearboxRotationsToHeightMM / 1000.0, MetersPerSecond));
    followerLog.voltage(followerMotor.getVoltage());
    followerLog.current(followerMotor.getCurrent());
  }

  public void logShoulder(SysIdRoutineLog log) {
    var motorLog = log.motor("Shoulder");
    var motor = shoulder.getMotor();
    motorLog.angularPosition(Angle.ofBaseUnits(motor.getPosition(), Rotations));
    motorLog.angularVelocity(AngularVelocity.ofBaseUnits(motor.getVelocity(), RotationsPerSecond));
    motorLog.voltage(motor.getVoltage());
    motorLog.current(motor.getCurrent());
  }

  public void logWrist(SysIdRoutineLog log) {
    var motorLog = log.motor("Wrist");
    var motor = wrist.getMotor();
    motorLog.angularPosition(Angle.ofBaseUnits(motor.getPosition(), Rotations));
    motorLog.angularVelocity(AngularVelocity.ofBaseUnits(motor.getVelocity(), RotationsPerSecond));
    motorLog.voltage(motor.getVoltage());
    motorLog.current(motor.getCurrent());
  }

  public void recordElevatorState(State state) {
    routinePublisher.set("Elevator " + state.name());
  }

  public void recordShoulderState(State state) {
    routinePublisher.set("Shoulder " + state.name());
  }

  public void recordWristState(State state) {
    routinePublisher.set("Wrist " + state.name());
  }

  //**************************** Kinematics ****************************//

  public double getElevatorHeightMM() {
    return leadMotor.getPosition() * Constants.Elevator.gearboxRotationsToHeightMM;
  }

  public double[] elevatorForwardKinematics() {
    double x, y;
    x =
        (Math.cos(shoulderSetpoint) * Constants.Shoulder.shoulderArmLengthMM)
            + (Math.cos(wristSetpoint) * Constants.Wrist.chuteExitXOffsetMM)
            - (Math.sin(wristSetpoint) * Constants.Wrist.chuteExitYOffsetMM);
    y =
        Constants.Elevator.baseHeightMM
            + (elevatorSetpoint * Constants.Elevator.gearboxRotationsToHeightMM)
            + (Math.sin(shoulderSetpoint) * Constants.Shoulder.shoulderArmLengthMM)
            + (Math.cos(wristSetpoint) * Constants.Wrist.chuteExitYOffsetMM)
            + (Math.sin(wristSetpoint) * Constants.Wrist.chuteExitXOffsetMM);
    return new double[] {x, y};
  }

  public boolean shoulderUnderLoad() {
    return shoulder.underLoad();
  }

  public boolean wristUnderLoad() {
    return wrist
    .underLoad();
  }

  //**************************** Encoder stuff ****************************//

  public double getElevatorRotations() {
    return leadMotor.getPosition();
  }

  public double getElevatorRadians() {
    return leadMotor.getPosition() * 2 * Math.PI;
  }

  public double getShoulderRadians() {
    return shoulder.getPositionRadians();
  }

  public double getWristRadians() {
    return wrist.getPositionRadians();
  }

    /***
   * Returns the difference between the right encoder and left encoder positions
   * @return Right pos - Left pos
   */
  public double getEncoderDiff() {
    return leadMotor.getPosition() - followerMotor.getPosition();
  }

  public boolean isElevatorAtLimits() {
    return getElevatorRotations() <= 0 || getElevatorRotations() >= Constants.Elevator.maxExtensionRotations;
  }

  public boolean isShoulderAtLimits() {
    return getShoulderRadians() <= Constants.Shoulder.shoulderHardStopMin || getShoulderRadians() >= Constants.Shoulder.shoulderHardStopMax;
  }

  public boolean isWristAtLimits() {
    return getWristRadians() <= Constants.Wrist.wristMin || getWristRadians() >= Constants.Wrist.wristMax;
  }

  //**************************** PID stuff ****************************//

  public boolean atElevatorReference() {
    return Math.abs(leadMotor.getPosition() - elevatorSetpoint) < 0.05;
  }

  public boolean atShoulderReference() {
    return Math.abs(shoulder.getPositionRadians() - shoulderSetpoint)
        < 0.035; // 2 degrees tolerance // 0.087 = 5 degrees
  }

  public boolean atWristReference() {
    return Math.abs(wrist.getPositionRadians() - wristSetpoint) < 0.035; // 2 degrees tolerance
  }

  //************************************************ Setters ************************************************//

  //**************************** Manual ****************************//

  public void setElevatorVoltage(Voltage value) {
    leadMotor.setControl(value.in(Volts), ControlType.kVoltage);
  }

  public void setShoulderVoltage(Voltage value) {
    shoulder.setVoltage(value.in(Volts));
  }

  public void setWristVoltage(Voltage value) {
    wrist.setVoltage(value.in(Volts));
  }

  //**************************** PID ****************************//


  /***
   * Set the elevator target
   * @param position target position in rotations
   */
  public void set(double position) {
    // Constants.log("Elevator target position:" + position);

    elevatorSetpoint = position;
    elevatorSetpoint =
        MathUtil.clamp(
            elevatorSetpoint, Constants.Elevator.minExtensionRotations, Constants.Elevator.maxExtensionRotations);
    leadMotor.setControl(elevatorSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public void setL4Mode(boolean value) {
    isScoringL4 = value;
  }

  public void setElevatorHeightMM(double heightMM) {
    set(heightMM / Constants.Elevator.gearboxRotationsToHeightMM);
  }

  public void setElevatorAdditive(double additive) {
    if (additive == 0) return;
    // Constants.log(elevatorSetpoint);
    elevatorSetpoint += additive;
    set(elevatorSetpoint);
    SmartDashboard.putNumber("Elevator setpoint", elevatorSetpoint);
  }

  /***
   * Set the shoulder target in RADIANS
   * @param position
   */
  public void setShoulder(double position) {
    shoulderSetpoint =
        MathUtil.clamp(position, Constants.Shoulder.shoulderMin, Constants.Shoulder.shoulderMax);
    shoulder.setpoint(position);
    checkWrist();
  }

  public void setShoulderAdditive(double additive) {
    if (additive == 0) return;
    // Constants.log(shoulderSetpoint + (additive * 0.02));
    setShoulder(shoulderSetpoint + (additive * 0.02));
  }

  public void driveShoulder(double dutyCycle) {
    shoulder.set(dutyCycle);
  }

  public void checkWrist() {
    // Constants.log(shoulderSetpoint + Constants.Wrist.wristMinShoulderOffsetRotations);
    // Constants.log(wristSetpoint);
    if (leadMotor.getPosition() < Constants.Elevator.hardStopExt) {
      wrist.setpoint(wristSetpoint);
    } else {
      wrist.setpoint(
        MathUtil.clamp(
            wristSetpoint,
            Constants.Wrist.startingRotationRadians,
            Constants.Wrist.wristMax));
    }

    SmartDashboard.putNumber("Wrist setpoint", elevatorSetpoint);
  }

  /***
   * Set the wrist target in RADIANS
   * @param position
   */
  public void setWrist(double position) {
    wristSetpoint = MathUtil.clamp(position, Constants.Wrist.wristMin, Constants.Wrist.wristMax);
    checkWrist();
  }

  public void setWristAdditive(double additive) {
    if (additive == 0) return;
    setWrist(wristSetpoint + (additive * 0.02));
  }

  //**************************** Kinematics ****************************//

  /***
   * Tell the end pivot of the claw to go to a target height and extension in MM. Deprecated - wrist mech changed
   * @param heightMM Height of coral chute
   * @param extensionMM Center of coral chute
   * @param wristAngleDegrees Wrist angle (0 parallel to floor, chute facing horizontal)
   */
  @Deprecated
  public void goToPosition(
      double extensionMM, double heightMM, double wristAngleDegrees, boolean verboseLogging) {
    Constants.log("Going to position " + heightMM + " " + extensionMM);
    kinematicsTarget = new double[] { extensionMM, heightMM };

    if (heightMM < 200 || heightMM > 1700) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }

    double relativeHeightMM = heightMM - Constants.Elevator.baseHeightMM;
    double wristRad = Math.toRadians(wristAngleDegrees);
    double x =
        Constants.Wrist.chuteExitXOffsetMM * Math.cos(wristRad)
            - (Constants.Wrist.chuteExitYOffsetMM * Math.sin(wristRad));
    double y =
        Constants.Wrist.chuteExitYOffsetMM * Math.cos(wristRad)
            + (Constants.Wrist.chuteExitXOffsetMM * Math.sin(wristRad));
    double requiredX = extensionMM - x;
    double shoulderRotRadians =
        Math.acos(requiredX / Constants.Shoulder.shoulderArmLengthMM); // Always positive

    if (verboseLogging) {
      Constants.log("Wrist fwd kinematics from required angle (x, y): " + x + " " + y);
      Constants.log(
          "Shoulder required x extension = "
              + requiredX
              + ", required rotation in degrees is either "
              + Math.toDegrees(shoulderRotRadians)
              + " or "
              + Math.toDegrees(-shoulderRotRadians));
    }

    if (Double.isNaN(shoulderRotRadians)) {
      Constants.log("Can't reach target!");
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }
    if (relativeHeightMM < Constants.Elevator.maxHeightExtensionMM) {
      // Prioritize elevator movement
      shoulderRotRadians = -shoulderRotRadians;
    }

    if (verboseLogging) {
      Constants.log(
          "Shoulder rotation is decided to be "
              + Math.toDegrees(shoulderRotRadians)
              + " degrees ("
              + shoulderRotRadians
              + ") radians");
    }

    // Constants.log("Shoulder rotation: " + shoulderRotRadians);
    // Constants.log("X total Extension: " + (x + (Math.cos(shoulderRotRadians) *
    // Constants.Shoulder.shoulderArmLengthMM)));
    // Constants.log("Elevator + wrist Y: " + (y + (Math.sin(shoulderRotRadians) *
    // Constants.Shoulder.shoulderArmLengthMM)));

    double requiredElevatorExt =
        (relativeHeightMM
            - (y + (Math.sin(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));

    if (verboseLogging) {
      Constants.log(
          "Required elevator extension is "
              + requiredElevatorExt
              + " mm, or "
              + requiredElevatorExt / Constants.Elevator.gearboxRotationsToHeightMM
              + " sprocket rotations");
    }

    if (requiredElevatorExt < 0) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }

    // Constants.log("Successful!");

    wristSetpoint = wristRad;
    shoulderSetpoint = shoulderRotRadians;
    setWrist(wristSetpoint);
    setShoulder(shoulderSetpoint);
    setElevatorHeightMM(elevatorSetpoint);
  }

  //************************************************ Config & state stuff ************************************************//

  public void resetEncoders() {
    Constants.log("Resetting encoders...");
    leadMotor.setEncoder(Constants.Elevator.minExtensionRotations);
    shoulderMotor.setEncoderRadians(Constants.Shoulder.startingRotationRadians);
    wristMotor.setEncoderRadians(Constants.Wrist.startingRotationRadians);
    elevatorSetpoint = Constants.Elevator.minExtensionRotations;
    shoulderSetpoint = Constants.Shoulder.startingRotationRadians;
    wristSetpoint = Constants.Wrist.startingRotationRadians;
  }

  public void reconfigure() {
    Constants.log(
        SmartDashboard.getNumber("Elevator Kp", 0.4)
            + " "
            + SmartDashboard.getNumber("Elevator Ki", 0.00001)
            + " "
            + SmartDashboard.getNumber("Elevator Kd", 0));
    leadMotor.configurePID(
        SmartDashboard.getNumber("Elevator Kp", 0.4),
        SmartDashboard.getNumber("Elevator Ki", 0.00001),
        SmartDashboard.getNumber("Elevator Kd", 0));
    shoulder.reconfigure();
    wrist.reconfigure();
  }

  public void recalibrateArmAbsoluteEncoder() {
    double offset = Constants.Shoulder.encoderOffset;
    Constants.log(
        "Current shoulder position degrees: " + Math.toDegrees(shoulder.getPositionRadians()));
    double diff = -78.0 - Math.toDegrees(shoulder.getPositionRadians());
    Constants.log(
        "Calculated difference in degrees: " + diff + ", in encoder rotations: " + diff / 360);
    Constants.log("Old encoder offset: " + offset);
    offset += diff / 360;
    Constants.log("New encoder offset: " + offset);
  }

  private int loop = 0;
  public boolean armDisabled = false;

  @Override
  public void periodic() {
    // if (shoulder.getPositionRadians() > 0 && wrist.getPositionRadians() > 0) isScoringL4 = true;
    // else isScoringL4 = false;
    shoulder.periodic();
    wrist.setOffsetAngleRadians(shoulder.getPositionRadians());
    wrist.periodic();

    if (leadMotor.getPosition() < 0.5 && shoulder.getPositionRadians() < Math.toRadians(-50)) {

    }
    /*loop++;
    if (loop > 100) {
      Constants.log("Shoulder absolute position degrees: " + Math.toDegrees(shoulder.getPositionRadians()));
      loop = 0;
    }*/
    if (shoulder.getPositionRadians() < Constants.Shoulder.shoulderHardStopMin) {
      // armDisabled = true;
      //Constants.log("Shoulder going behind hard stop min");
      // shoulder.stop();
    }
    if (shoulder.getPositionRadians() > Constants.Shoulder.shoulderHardStopMax) {
      //Constants.log("Shoulder going past hard stop max");
      // shoulder.stop();
    }
    /*if (shoulder.getVelocityRadPS() > Math.PI * 5) {
      Constants.log("Shoulder going too fast");
      shoulder.stop();
    }*/

    SmartDashboard.putNumber("Elevator current rotation", leadMotor.getPosition());
    SmartDashboard.putNumber(
        "Elevator current extension MM",
        leadMotor.getPosition() * Constants.Elevator.gearboxRotationsToHeightMM);
    followerMotor.publishToNetworkTables();
    leadMotor.publishToNetworkTables();
    shoulderMotor.publishToNetworkTables();
    wristMotor.publishToNetworkTables();
    setpointPublisher.set(elevatorSetpoint);
    double[] kinematics = elevatorForwardKinematics();
    kinematicsPublisher.set(kinematics);
    inverseKinematicsPublisher.set(kinematicsTarget);
  }

  public void logEncoders() {
    leadMotor.logEncoderState();
    followerMotor.logEncoderState();
  }
}
