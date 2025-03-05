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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardSparkMax;

public class Elevator extends SubsystemBase {
  private HazardSparkMax leftMotor;
  private HazardSparkMax rightMotor;

  //private SparkMax elevatorWrist; // Not in CAN yet
  //private SparkClosedLoopController wristMotorController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  //private RelativeEncoder wristEncoder;
  private PIDController elevatorPID;
  private ElevatorFeedforward elevatorFeedforward;
  private PIDController wristPID;

  private double elevatorSetpoint = 0;
  private double wristSetpoint = 0;

  private NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
  private DoublePublisher setpointPublisher = elevatorTable.getDoubleTopic("Elevator setpoint").publish();

  public Elevator() {
    //leftMotor = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless);
    //rightMotor = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless);
    // elevatorWrist = new SparkMax(Constants.Elevator.elevatorWrist, MotorType.kBrushless);


    // Default configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig shoulderConfig = new SparkMaxConfig();

    leftConfig.smartCurrentLimit(Constants.Elevator.elevatorCurrentLimit);
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.follow(Constants.Elevator.right, true);
    leftConfig.absoluteEncoder.positionConversionFactor(
        Constants.Elevator.gearboxReduction);
    leftConfig.absoluteEncoder.velocityConversionFactor(
        Constants.Elevator.gearboxReduction);

    rightConfig.smartCurrentLimit(Constants.Elevator.elevatorCurrentLimit);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.closedLoop.outputRange(-1, 1);
    rightConfig.closedLoop.iMaxAccum(1);
    rightConfig.closedLoop.pid(0.4, 0, 0);
    rightConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    rightConfig.closedLoop.maxMotion.allowedClosedLoopError(1);
    rightConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    rightConfig.closedLoop.maxMotion.maxAcceleration(80, ClosedLoopSlot.kSlot0); //TODO:
    rightConfig.closedLoop.maxMotion.maxVelocity(5000, ClosedLoopSlot.kSlot0);
    rightConfig.closedLoop.iZone(15);
    rightConfig.closedLoop.feedbackSensor(
        FeedbackSensor.kPrimaryEncoder);
    rightConfig.absoluteEncoder.positionConversionFactor(
        Constants.Elevator.gearboxReduction);
    rightConfig.absoluteEncoder.velocityConversionFactor(
        Constants.Elevator.gearboxReduction);

    shoulderConfig.inverted(true);
    shoulderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shoulderConfig.absoluteEncoder.positionConversionFactor(Constants.Shoulder.gearboxReduction);
    shoulderConfig.absoluteEncoder.velocityConversionFactor(Constants.Shoulder.gearboxReduction);

    leftMotor = new HazardSparkMax(Constants.Elevator.left, MotorType.kBrushless, leftConfig, false, true, elevatorTable);
    rightMotor = new HazardSparkMax(Constants.Elevator.right, MotorType.kBrushless, rightConfig, false, true, elevatorTable);


    //leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rightMotor.configure(
    //    rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    // elevatorWrist.configure(
    //   wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //leftEncoder = leftMotor.getEncoder();
    //rightEncoder = rightMotor.getEncoder();
    //wristEncoder = elevatorWrist.getAlternateEncoder();

    elevatorPID = new PIDController(0.1, 0, 0.05);
    elevatorFeedforward = new ElevatorFeedforward(0, 0.4128, 0, 0); // See
    wristPID = new PIDController(0.1, 0, 0);

    //leftMotorController = leftMotor.getClosedLoopController();
    //rightMotorController = rightMotor.getClosedLoopController();
    //wristMotorController = elevatorWrist.getClosedLoopController();    
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
    Constants.log("Elevator target position:" + position); 
    
    elevatorSetpoint = position;
    if (elevatorSetpoint > Constants.Elevator.minExtension) elevatorSetpoint = Constants.Elevator.minExtension;
    if (elevatorSetpoint < Constants.Elevator.maxExtension) elevatorSetpoint = Constants.Elevator.maxExtension;

    rightMotor.setControl(elevatorSetpoint, ControlType.kMAXMotionPositionControl);
  }

  public void setElevatorHeight(double heightMM) {
    set(
        heightMM
            / (Constants.Elevator.gearboxRotationsToHeightMM
                * Constants.Elevator.gearboxReduction));
  }

  public void setAdditive(double additive) {
    elevatorSetpoint += additive;
    set(elevatorSetpoint);
  }

  public void setWrist(double position) {
    wristSetpoint = position;
    wristPID.setSetpoint(wristSetpoint);
  }

  public void setWristAdditive(double additive) {
    wristSetpoint = wristPID.getSetpoint() + additive;
    wristPID.setSetpoint(wristSetpoint);
  }

  /***
   * Tell the end pivot of the claw to go to a target height and extension in MM
   * @param heightMM
   * @param extensionMM
   */
  public void goToPosition(double heightMM, double extensionMM) {
    double clawRotation = Math.acos(extensionMM / 300);
    double elevatorH = heightMM - Math.sin(clawRotation) * 300;

    //wristMotorController.setReference(clawRotation * Constants.Elevator.wristGearboxReduction, ControlType.kMAXMotionPositionControl);
    setElevatorHeight(elevatorH);
  }

  public double getWrist() {
    return 0;
    //return wristEncoder.getPosition();
  }

  private int loop = 0;

  @Override
  public void periodic() {
    leftMotor.publishToNetworkTables();
    rightMotor.publishToNetworkTables();
    setpointPublisher.set(elevatorSetpoint);

    loop++;
    if (loop > 50) { // Log twice per second
      rightMotor.logEncoderState();
      // Constants.log("Calculated:" + elevatorPID.calculate(leftEncoder.getPosition()));
      // Constants.log("Encoder:" + leftEncoder.getPosition());
      loop = 0;
    }
  }

  public void logEncoders() {
    leftMotor.logEncoderState();
    rightMotor.logEncoderState();
  }
}
