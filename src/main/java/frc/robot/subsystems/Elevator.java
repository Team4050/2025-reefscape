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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMax elevatorWrist; // Not in CAN yet
  private SparkClosedLoopController leftMotorController;
  private SparkClosedLoopController rightMotorController;
  private SparkClosedLoopController wristMotorController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder wristEncoder;
  private PIDController elevatorPID;
  private ElevatorFeedforward elevatorFeedforward;
  private PIDController wristPID;
  private ArmFeedforward wristFeedforward;

  private double elevatorTarget = 0;
  private double wristTarget = 0;

  private boolean useNetworkTables;
  private NetworkTable elevatorTable;
  private DoublePublisher motorVelocityRads;
  private DoublePublisher appliedOutputAmps;
  private DoublePublisher encoderPosition;
  private DoublePublisher setpointPublisher;

  public Elevator(boolean useNetworkTables) {
    this.useNetworkTables = useNetworkTables;
    leftMotor = new SparkMax(Constants.Elevator.left, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.right, MotorType.kBrushless);
    elevatorWrist = new SparkMax(Constants.Elevator.shoulder, MotorType.kBrushless);

    // Default configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    leftConfig.smartCurrentLimit(60);
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.follow(Constants.Elevator.right, true);
    leftConfig.absoluteEncoder.positionConversionFactor(
        Constants.Elevator.elevatorGearboxReduction);
    leftConfig.absoluteEncoder.velocityConversionFactor(
        Constants.Elevator.elevatorGearboxReduction);

    rightConfig.smartCurrentLimit(60);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.closedLoop.outputRange(-1, 1);
    rightConfig.closedLoop.iMaxAccum(1);
    rightConfig.closedLoop.pidf(0.7, 0, 0.05, 0);
    rightConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    rightConfig.closedLoop.maxMotion.maxAcceleration(0, ClosedLoopSlot.kSlot0);
    rightConfig.closedLoop.maxMotion.maxVelocity(0, ClosedLoopSlot.kSlot0);
    rightConfig.closedLoop.feedbackSensor(
        FeedbackSensor.kPrimaryEncoder);
    rightConfig.absoluteEncoder.positionConversionFactor(
        Constants.Elevator.elevatorGearboxReduction);
    rightConfig.absoluteEncoder.velocityConversionFactor(
        Constants.Elevator.elevatorGearboxReduction);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorWrist.configure(
       wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    wristEncoder = elevatorWrist.getAlternateEncoder();
    leftMotorController = leftMotor.getClosedLoopController();
    rightMotorController = rightMotor.getClosedLoopController();
    elevatorPID = new PIDController(0.1, 0, 0.05);
    elevatorFeedforward = new ElevatorFeedforward(0, 0.00262, 0, 0);
    // 0.0036 for 40A limit
    // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html and my math

    wristConfig.inverted(true);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    wristConfig.closedLoop.outputRange(-0.2, 0.2);
    wristConfig.closedLoop.iMaxAccum(1);
    wristConfig.closedLoop.pidf(0, 0, 0, 0);
    wristConfig.absoluteEncoder.positionConversionFactor(Constants.Elevator.shoulderGearboxReduction);
    wristConfig.absoluteEncoder.velocityConversionFactor(Constants.Elevator.shoulderGearboxReduction);
    wristMotorController = elevatorWrist.getClosedLoopController();
    wristPID = new PIDController(0.1, 0, 0);
    wristFeedforward = new ArmFeedforward(0, 0, 0, 0);

    if (useNetworkTables) {
      elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
      motorVelocityRads = elevatorTable.getDoubleTopic("Motor velocity - Revs").publish();
      appliedOutputAmps = elevatorTable.getDoubleTopic("Stator current - Amps").publish();
      encoderPosition = elevatorTable.getDoubleTopic("Encoder position - Revs").publish();
      setpointPublisher = elevatorTable.getDoubleTopic("PID setpoint - Revs").publish();
    }
  }

  /**
   * Height in mm to encoder revolutions of elevator motors
   *
   * @param h
   * @return
   */
  public double elevatorHeightMMToEncoder(double h) {
    return (h / Constants.Elevator.elevatorGearboxRotationsToHeightMM)
        / Constants.Elevator.elevatorGearboxReduction;
  }

  /**
   * Returns wrist angle, looking at motor output face, from starting config in radians
   *
   * @param r
   * @return The wrist angle in radians
   */
  public double encoderRevToWristAngle(double r) {
    r /= Constants.Elevator.shoulderEncoderCountsPerRevolution;
    r /= Constants.Elevator.shoulderGearboxReduction;
    return r * 2 * Math.PI;
  }

  public void set(double position) {
    // Constants.log("Elevator target position:" + position);
    // leftMotor.set(position); //
    // Constants.log(position);
    elevatorTarget = position;
    if (elevatorTarget > 0) elevatorTarget = 0;
    if (elevatorTarget < -104) elevatorTarget = -104;
    rightMotorController.setReference(
        elevatorTarget,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        Constants.Elevator.elevatorFFVoltage);
  }

  public void setElevatorHeight(double heightMM) {
    set(
        heightMM
            / (Constants.Elevator.elevatorGearboxRotationsToHeightMM
                * Constants.Elevator.elevatorGearboxReduction));
  }

  public void setAdditive(double additive) {
    elevatorTarget += additive;
    set(elevatorTarget);
  }

  public void setWrist(double position) {
    wristTarget = position;
    wristPID.setSetpoint(wristTarget);
  }

  public void setWristAdditive(double additive) {
    wristTarget = wristPID.getSetpoint() + additive;
    wristPID.setSetpoint(wristTarget);
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

  /***
   * Returns the difference between the right encoder and left encoder positions
   * @return Right pos - Left pos
   */
  public double getEncoderDiff() {
    return rightEncoder.getPosition() - leftEncoder.getPosition();
  }

  public double getWrist() {
    return 0;
    //return wristEncoder.getPosition();
  }

  private int loop = 0;

  @Override
  public void periodic() {

    // leftMotor.set(pidElevator.calculate(encoderLeft.getPosition())); // Use for alternate control
    // elevatorWrist.set(pidWrist.calculate(wristEncoder.getPosition()));
    //Constants.log(wristFeedforward.calculate(wristEncoder.getPosition(), wristEncoder.getVelocity()));

    if (useNetworkTables) {
      motorVelocityRads.set(rightEncoder.getVelocity());
      appliedOutputAmps.set(rightMotor.getOutputCurrent());
      encoderPosition.set(rightEncoder.getPosition());
      setpointPublisher.set(elevatorTarget);
    }
    
    loop++;
    if (loop > 25) { // Log twice per second
      // Constants.log("Raw encoder L R:" + leftEncoder.getPosition() + rightEncoder.getPosition());
      // Constants.log("Calculated:" + elevatorPID.calculate(leftEncoder.getPosition()));
      Constants.log(
          "Output duty cycle:"
              + leftMotor.getAppliedOutput()
              + " "
              + rightMotor.getAppliedOutput());
      // Constants.log("Encoder:" + leftEncoder.getPosition());
      loop = 0;
    }
  }
}
