package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private PIDController shoulderPID;
  private ArmFeedforward shoulderFF;

  private double elevatorSetpoint = 0;
  private double shoulderSetpoint = 0;
  private double wristSetpoint = 0;

  private NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
  private DoublePublisher setpointPublisher = elevatorTable.getDoubleTopic("Elevator setpoint").publish();
  private DoubleArrayPublisher kinematicsPublisher = elevatorTable.getDoubleArrayTopic("Elevator kinematics").publish();
  private double PSub = 0.25;
  private double ISub = 0.005;
  private double DSub = 0.05;

  public Elevator() {
    SmartDashboard.putNumber("P", PSub);
    SmartDashboard.putNumber("I", ISub);
    SmartDashboard.putNumber("D", DSub);

    shoulderFF= new ArmFeedforward(0, 1, 0);
    //leftMotor = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless);
    //rightMotor = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless);
    // elevatorWrist = new SparkMax(Constants.Elevator.elevatorWrist, MotorType.kBrushless);


    // Default configs
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
    leadConfig.closedLoop.iMaxAccum(1);
    leadConfig.closedLoop.pid(0.4, 0, 0);
    leadConfig.closedLoop.maxMotion.allowedClosedLoopError(0.01);
    leadConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    leadConfig.closedLoop.maxMotion.maxAcceleration(30, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.maxMotion.maxVelocity(100, ClosedLoopSlot.kSlot0);
    leadConfig.closedLoop.iZone(15);
    leadConfig.closedLoop.feedbackSensor(
        FeedbackSensor.kPrimaryEncoder);
    leadConfig.encoder.positionConversionFactor(
        Constants.Elevator.gearboxReduction);
    leadConfig.encoder.velocityConversionFactor(
        Constants.Elevator.gearboxReduction);

    shoulderConfig.smartCurrentLimit(Constants.Shoulder.currentLimit);
    shoulderConfig.idleMode(IdleMode.kBrake);
    shoulderConfig.inverted(true);
    shoulderConfig.closedLoop.pidf(0.25, 0.005, 0.05, 0.1);
    shoulderConfig.closedLoop.iMaxAccum(0.085);
    shoulderConfig.closedLoop.outputRange(-0.1, 0.1);
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
    wristConfig.closedLoop.maxMotion.allowedClosedLoopError(0.05);
    wristConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    wristConfig.encoder.positionConversionFactor(Constants.Wrist.gearboxReduction);
    wristConfig.encoder.velocityConversionFactor(Constants.Wrist.gearboxReduction);

    backMotor = new HazardSparkMax(Constants.Elevator.back, MotorType.kBrushless, leadConfig, false, true, elevatorTable);
    frontMotor = new HazardSparkMax(Constants.Elevator.front, MotorType.kBrushless, followConfig, false, false, elevatorTable);
    shoulderMotor = new HazardSparkMax(Constants.Shoulder.CAN, MotorType.kBrushless, shoulderConfig, true, "Shoulder");
    shoulderMotor.setEncoder(Constants.Shoulder.shoulderStartingRotation);
    shoulderSetpoint = Constants.Shoulder.shoulderStartingRotation;
    wristMotor = new HazardSparkMax(Constants.Wrist.CAN, MotorType.kBrushless, wristConfig, true, "Wrist");
    wristMotor.setEncoder(Constants.Wrist.startingPositionRotation);
    wristSetpoint = 0;

    elevatorPID = new PIDController(0.1, 0, 0.05);//Unused, backup if manual feedforward calculation is needed
    elevatorFF = new ElevatorFeedforward(0, 0.4128, 0, 0);

    shoulderPID = new PIDController(0.1, 0, 0); // Unused, backup if arm feedforward is needed
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

  public void set(double position) {
    //Constants.log("Elevator target position:" + position); 
    
    elevatorSetpoint = position;
    elevatorSetpoint = MathUtil.clamp(elevatorSetpoint, Constants.Elevator.minExtension, Constants.Elevator.maxExtension);
    backMotor.setControl(elevatorSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public void setElevatorHeightMM(double heightMM) {
    set(heightMM / Constants.Elevator.gearboxRotationsToHeightMM);
  }

  public void setAdditive(double additive) {
    elevatorSetpoint += additive;
    set(elevatorSetpoint);
  }

  public void setShoulder(double position) {
    shoulderSetpoint = MathUtil.clamp(position, Constants.Shoulder.shoulderMin, Constants.Shoulder.shoulderMax);
    //Constants.log("Setpoint: " + shoulderSetpoint);
    //Constants.log("Position: " + shoulderMotor.getPosition());
    shoulderMotor.setControl(shoulderSetpoint, ControlType.kPosition);
    checkWrist();
  }

  public void checkWrist() {
    wristMotor.setControl(
      MathUtil.clamp(wristSetpoint, 
      shoulderSetpoint + Constants.Wrist.wristMinShoulderOffsetRotations, 
      shoulderSetpoint + Constants.Wrist.wristMaxShoulderOffsetRotations), ControlType.kMAXMotionPositionControl);
  }

  public void setWrist(double position) {
    wristSetpoint = MathUtil.clamp(position, Constants.Wrist.wristMin, Constants.Wrist.wristMax);
    //Constants.log(wristSetpoint);
    checkWrist();
  }

  /***
   * Tell the end pivot of the claw to go to a target height and extension in MM
   * @param heightMM Height of coral chute
   * @param extensionMM Center of coral chute
   * @param wristAngleDegrees Wrist angle (0 parallel to floor, chute facing horizontal)
   */
  public void goToPosition(double heightMM, double extensionMM, double wristAngleDegrees) {
    Constants.log("Going to position " + heightMM + " " + extensionMM);

    if (heightMM < 200) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }
    
    double relativeHeightMM = heightMM - Constants.Elevator.baseHeightMM;
    double wristRad = Math.toRadians(wristAngleDegrees);
    double x = Constants.Wrist.chuteCenterXOffsetMM * Math.cos(wristRad) - (Constants.Wrist.chuteCenterYOffsetMM * Math.sin(wristRad));
    //Constants.log("Wrist x: " + x);
    double y = Constants.Wrist.chuteCenterYOffsetMM * Math.cos(wristRad) + (Constants.Wrist.chuteCenterXOffsetMM * Math.sin(wristRad));
    //Constants.log("Wrist y:" + y);
    double requiredX = extensionMM - x;
    //Constants.log("Required X extension from arm: " + requiredX);
    double shoulderRotRadians = Math.acos(requiredX / Constants.Shoulder.shoulderArmLengthMM); //Always positive

    if (Double.isNaN(shoulderRotRadians)) {
      Constants.log("Can't reach target!");
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }
    if (relativeHeightMM < Constants.Elevator.maxHeightExtensionMM) {
      // Prioritize elevator movement
      shoulderRotRadians = -shoulderRotRadians;
    }

    //Constants.log("Shoulder rotation: " + shoulderRotRadians);
    //Constants.log("X total Extension: " + (x + (Math.cos(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));
    //Constants.log("Elevator + wrist Y: " + (y + (Math.sin(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));

    double requiredElevatorExt = (relativeHeightMM - (y + (Math.sin(shoulderRotRadians) * Constants.Shoulder.shoulderArmLengthMM)));
    //Constants.log("Required elevator height: " + requiredElevatorExt);
    Constants.log(requiredElevatorExt + Constants.Elevator.baseHeightMM);

    if (requiredElevatorExt < 0) {
      DriverStation.reportError("Tried to reach an impossible location with arm subsystem!", false);
      return;
    }

    //Constants.log("Successful!");

    //wristMotorController.setReference(clawRotation * Constants.Elevator.wristGearboxReduction, ControlType.kMAXMotionPositionControl);
    wristSetpoint = wristRad / (2.0 * Math.PI);
    shoulderSetpoint = shoulderRotRadians / (2.0 * Math.PI);
    elevatorSetpoint = requiredElevatorExt;
    setWrist(wristSetpoint);
    setShoulder(shoulderSetpoint);
    setElevatorHeightMM(elevatorSetpoint);
  }

  /***
   * Returns the difference between the right encoder and left encoder positions
   * @return Right pos - Left pos
   */
  public double getEncoderDiff() {
    return backMotor.getPosition() - frontMotor.getPosition();
  }

  public void reconfig() {
    Constants.log(lastP + " " + lastI + " " + lastD);
    shoulderMotor.configurePID(SmartDashboard.getNumber("P", PSub), SmartDashboard.getNumber("I", ISub), SmartDashboard.getNumber("D", DSub));
  }

  double lastP = 0;
  double lastI = 0;
  double lastD = 0;

  @Override
  public void periodic() {
    shoulderFF.calculate(shoulderSetpoint, shoulderMotor.getVelocity());

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
