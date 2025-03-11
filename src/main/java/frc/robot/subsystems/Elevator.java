package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardArm;
import frc.robot.hazard.HazardSparkMax;

public class Elevator extends SubsystemBase {
  private HazardSparkMax backMotor;
  private HazardSparkMax frontMotor;
  private HazardSparkMax shoulderMotor;
  private HazardSparkMax wristMotor;

  //private SparkMax elevatorWrist; // Not in CAN yet
  //private SparkClosedLoopController wristMotorController;
  private PIDController elevatorPID;
  private ElevatorFeedforward elevatorFF;

  private HazardArm shoulder;
  private HazardArm wrist;

  private double elevatorSetpoint = 0;
  private double shoulderSetpoint = 0;
  private double wristSetpoint = 0;

  private NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
  private DoublePublisher setpointPublisher = elevatorTable.getDoubleTopic("Elevator setpoint").publish();
  private DoubleArrayPublisher kinematicsPublisher = elevatorTable.getDoubleArrayTopic("Elevator kinematics").publish();

  public Elevator(boolean tuningMode) {
    SparkMaxConfig followConfig = new SparkMaxConfig();
    SparkMaxConfig leadConfig = new SparkMaxConfig();
    SparkMaxConfig shoulderConfig = new SparkMaxConfig();
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    followConfig.smartCurrentLimit(Constants.Elevator.elevatorCurrentLimit);
    followConfig.idleMode(IdleMode.kBrake);
    followConfig.follow(Constants.Elevator.front, true);
    /*followConfig.absoluteEncoder.positionConversionFactor( //TODO: Change position conversion factor on sparkMaxes for convenience
        Constants.Elevator.gearboxReduction);
    followConfig.absoluteEncoder.velocityConversionFactor(
        Constants.Elevator.gearboxReduction);*/

    leadConfig.smartCurrentLimit(Constants.Elevator.elevatorCurrentLimit);
    leadConfig.idleMode(IdleMode.kBrake);
    leadConfig.inverted(true);
    leadConfig.closedLoop.outputRange(-1, 1);
    leadConfig.closedLoop.pid(0.3, 0.00001, 0.0001);
    leadConfig.closedLoop.iMaxAccum(0.01);
    leadConfig.closedLoop.maxMotion.allowedClosedLoopError(0.03);
    leadConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    leadConfig.closedLoop.maxMotion.maxAcceleration(500, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.maxMotion.maxVelocity(500, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.feedbackSensor(
        FeedbackSensor.kPrimaryEncoder);
    leadConfig.encoder.positionConversionFactor(
        Constants.Elevator.gearboxReduction);
    leadConfig.encoder.velocityConversionFactor(
        Constants.Elevator.gearboxReduction);

    shoulderConfig.smartCurrentLimit(Constants.Shoulder.currentLimit);
    shoulderConfig.idleMode(IdleMode.kBrake);
    shoulderConfig.inverted(true);
    shoulderConfig.closedLoop.pidf(0.25, 0.005, 0.05, 0.25);
    shoulderConfig.closedLoop.iMaxAccum(0.085);
    shoulderConfig.closedLoop.outputRange(-0.3, 0.3);
    shoulderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shoulderConfig.closedLoop.maxMotion.maxAcceleration(50);
    shoulderConfig.closedLoop.maxMotion.maxVelocity(100);
    shoulderConfig.closedLoop.maxMotion.allowedClosedLoopError(0.02);
    shoulderConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    shoulderConfig.encoder.positionConversionFactor(Constants.Shoulder.gearboxReduction);
    shoulderConfig.encoder.velocityConversionFactor(Constants.Shoulder.gearboxReduction);

    wristConfig.smartCurrentLimit(Constants.Shoulder.currentLimit);
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.inverted(true);
    wristConfig.closedLoop.pid(0.2, 0.00005, 0);
    wristConfig.closedLoop.iMaxAccum(5);
    wristConfig.closedLoop.outputRange(-0.4, 0.4);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    wristConfig.closedLoop.maxMotion.maxAcceleration(50);
    wristConfig.closedLoop.maxMotion.maxVelocity(100); //1 rotation/s
    wristConfig.closedLoop.maxMotion.allowedClosedLoopError(0.01);
    wristConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    wristConfig.encoder.positionConversionFactor(Constants.Wrist.gearboxReduction);
    wristConfig.encoder.velocityConversionFactor(Constants.Wrist.gearboxReduction);

    backMotor = new HazardSparkMax(Constants.Elevator.back, MotorType.kBrushless, leadConfig, false, true, elevatorTable);
    frontMotor = new HazardSparkMax(Constants.Elevator.front, MotorType.kBrushless, followConfig, false, false, elevatorTable);
    shoulderMotor = new HazardSparkMax(Constants.Shoulder.CAN, MotorType.kBrushless, shoulderConfig, true, "Shoulder");
    shoulderMotor.setEncoder(Constants.Shoulder.startingRotation);
    shoulderSetpoint = Constants.Shoulder.startingRotation;
    wristMotor = new HazardSparkMax(Constants.Wrist.CAN, MotorType.kBrushless, wristConfig, true, "Wrist");
    wristMotor.setEncoder(Constants.Wrist.startingRotation);
    wristSetpoint = 0;

    elevatorPID = new PIDController(0.1, 0, 0.05);//Unused, backup if manual feedforward calculation is needed
    elevatorFF = new ElevatorFeedforward(0, 0.4128, 0, 0);

    Constants.log("Shoulder NEO Kv: " + (49.0 / Constants.Shoulder.motor.KvRadPerSecPerVolt));

    shoulder = new HazardArm(
      shoulderMotor,
      0,
      0,//0.1,
      0,//1.0,//Constants.Shoulder.shoulderMotorTorqueNM / (Constants.Shoulder.motor.KtNMPerAmp * Constants.Shoulder.currentLimit) + 0.5,
      0,///49.0 / Constants.Shoulder.motor.KvRadPerSecPerVolt,
      0,
      0,
      0,
      1,
      0.1,
      "Shoulder",
      tuningMode);
    wrist = new HazardArm(
      wristMotor,
      0,
      0.1,
      0,
      0,
      0.2,
      0.00005,
      0,
      0,
      0,
      "Wrist",
      tuningMode);

    if (tuningMode) {
      SmartDashboard.putNumber("Elevator Kp", 0.35);
      SmartDashboard.putNumber("Elevator Ki", 0.00001);
      SmartDashboard.putNumber("Elevator Kd", 0);

      SmartDashboard.putData("Reconfigure controllers", new InstantCommand(() -> { this.reconfigure(); }, this));
    }
  }

  /**
   * Height in mm to encoder revolutions of elevator motors
   *
   * @param h
   * @return
   */
  public double elevatorHeightMMToEncoder(double h) {
    return (h / Constants.Elevator.gearboxRotationsToHeightMM)
        / Constants.Elevator.gearboxReduction;
  }

  /**
   * Returns wrist angle, looking at motor output face, from starting config in radians
   *
   * @param r
   * @return The wrist angle in radians
   */
  public double encoderRevToWristAngle(double r) {
    r /= Constants.Wrist.gearboxReduction;
    return r * 2 * Math.PI;
  }

  /***
   * Set the elevator target
   * @param position target position in rotations
   */
  public void set(double position) {
    //Constants.log("Elevator target position:" + position);

    elevatorSetpoint = position;
    elevatorSetpoint = MathUtil.clamp(elevatorSetpoint, Constants.Elevator.minExtension, Constants.Elevator.maxExtension);
    backMotor.setControl(elevatorSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public void setElevatorHeightMM(double heightMM) {
    set(heightMM / Constants.Elevator.gearboxRotationsToHeightMM);
  }

  public void setElevatorAdditive(double additive) {
    Constants.log(elevatorSetpoint);
    elevatorSetpoint += additive;
    set(elevatorSetpoint);
  }

  /***
   * Set the shoulder target in RADIANS
   * @param position
   */
  public void setShoulder(double position) {
    shoulderSetpoint = MathUtil.clamp(position, Constants.Shoulder.shoulderMin, Constants.Shoulder.shoulderMax);
    shoulder.setpoint(position);
    checkWrist();
  }

  public void setShoulderAdditive(double additive) {
    setShoulder(shoulderSetpoint + (additive * 0.02));
  }

  public void checkWrist() {
    //Constants.log(shoulderSetpoint + Constants.Wrist.wristMinShoulderOffsetRotations);
    wrist.setpoint(MathUtil.clamp(wristSetpoint,
    shoulderSetpoint + Constants.Wrist.wristMinShoulderOffsetRotations,
    shoulderSetpoint + Constants.Wrist.wristMaxShoulderOffsetRotations));
  }

  /***
   * Set the wrist target in RADIANS
   * @param position
   */
  public void setWrist(double position) {
    wristSetpoint = MathUtil.clamp(position, Constants.Wrist.wristMin, Constants.Wrist.wristMax);
    //Constants.log(wristSetpoint);
    checkWrist();
  }

  public void setWristAdditive(double additive) {
    setWrist(wristSetpoint + (additive * 0.02));
  }

  /***
   * Tell the end pivot of the claw to go to a target height and extension in MM
   * @param heightMM Height of coral chute
   * @param extensionMM Center of coral chute
   * @param wristAngleDegrees Wrist angle (0 parallel to floor, chute facing horizontal)
   */
  public void goToPosition(double heightMM, double extensionMM, double wristAngleDegrees, boolean verboseLogging) {
    Constants.log("Going to position " + heightMM + " " + extensionMM);

    if (heightMM < 200) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }

    double relativeHeightMM = heightMM - Constants.Elevator.baseHeightMM;
    double wristRad = Math.toRadians(wristAngleDegrees);
    double x = Constants.Wrist.chuteExitXOffsetMM * Math.cos(wristRad) - (Constants.Wrist.chuteExitYOffsetMM * Math.sin(wristRad));
    double y = Constants.Wrist.chuteExitYOffsetMM * Math.cos(wristRad) + (Constants.Wrist.chuteExitXOffsetMM * Math.sin(wristRad));
    double requiredX = extensionMM - x;
    double shoulderRotRadians = Math.acos(requiredX / Constants.Shoulder.shoulderArmLengthMM); //Always positive

    if (verboseLogging) {
      Constants.log("Wrist fwd kinematics from required angle (x, y): " + x + " " + y);
      Constants.log("Shoulder required x extension = " + requiredX + ", required rotation in degrees is either " + Math.toDegrees(shoulderRotRadians) + " or " + Math.toDegrees(-shoulderRotRadians));
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
      Constants.log("Shoulder rotation is decided to be " + Math.toDegrees(shoulderRotRadians) + " degrees (" + shoulderRotRadians + ") radians");
    }

    //Constants.log("Shoulder rotation: " + shoulderRotRadians);
    //Constants.log("X total Extension: " + (x + (Math.cos(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));
    //Constants.log("Elevator + wrist Y: " + (y + (Math.sin(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));

    double requiredElevatorExt = (relativeHeightMM - (y + (Math.sin(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));

    if (verboseLogging) {
      Constants.log("Required elevator extension is " + requiredElevatorExt + " mm, or " + requiredElevatorExt / Constants.Elevator.gearboxRotationsToHeightMM + " sprocket rotations");
    }

    if (requiredElevatorExt < 0) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }

    //Constants.log("Successful!");

    wristSetpoint = wristRad;
    shoulderSetpoint = shoulderRotRadians;
    setWrist(wristSetpoint);
    setShoulder(shoulderSetpoint);
    setElevatorHeightMM(elevatorSetpoint);
  }

  public double[] elevatorForwardKinematics() {
    double x, y;
    x = (Math.cos(shoulderSetpoint) * Constants.Shoulder.shoulderArmLengthMM) + (Math.cos(wristSetpoint) * Constants.Wrist.chuteExitXOffsetMM) - (Math.sin(wristSetpoint) * Constants.Wrist.chuteExitYOffsetMM);
    y = Constants.Elevator.baseHeightMM + (elevatorSetpoint * Constants.Elevator.gearboxRotationsToHeightMM) + (Math.sin(shoulderSetpoint) * Constants.Shoulder.shoulderArmLengthMM) + (Math.cos(wristSetpoint) * Constants.Wrist.chuteExitYOffsetMM) + (Math.sin(wristSetpoint) * Constants.Wrist.chuteExitXOffsetMM);
    return new double[] {x, y};
  }

  /***
   * Returns the difference between the right encoder and left encoder positions
   * @return Right pos - Left pos
   */
  public double getEncoderDiff() {
    return backMotor.getPosition() - frontMotor.getPosition();
  }

  public void resetEncoders() {
    Constants.log("Resetting encoders...");
    backMotor.setEncoder(Constants.Elevator.minExtension);
    shoulderMotor.setEncoder(Constants.Shoulder.startingRotation);
    wristMotor.setEncoder(Constants.Wrist.startingRotation);
  }

  public void reconfigure() {
    Constants.log(SmartDashboard.getNumber("Elevator Kp", 0.4) + " " + SmartDashboard.getNumber("Elevator Ki", 0.00001) + " " + SmartDashboard.getNumber("Elevator Kd", 0));
    backMotor.configurePID(SmartDashboard.getNumber("Elevator Kp", 0.4), SmartDashboard.getNumber("Elevator Ki", 0.00001), SmartDashboard.getNumber("Elevator Kd", 0));
    shoulder.reconfigure();
    wrist.reconfigure();
  }

  @Override
  public void periodic() {
    shoulder.periodic();
    wrist.periodic();

    frontMotor.publishToNetworkTables();
    backMotor.publishToNetworkTables();
    shoulderMotor.publishToNetworkTables();
    wristMotor.publishToNetworkTables();
    setpointPublisher.set(elevatorSetpoint);
    kinematicsPublisher.set(new double[] {elevatorSetpoint, shoulderSetpoint, wristSetpoint});
  }

  public void logEncoders() {
    backMotor.logEncoderState();
    frontMotor.logEncoderState();
  }
}
