package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  //private SparkMax elevatorWrist; //Not in CAN yet
  private SparkClosedLoopController leftMotorController;
  private SparkClosedLoopController rightMotorController;
  private AbsoluteEncoder encoderLeft;
  private AbsoluteEncoder encoderRight;
  private RelativeEncoder wristEncoder;
  private PIDController pidElevator;
  private PIDController pidWrist;

  private double elevatorTarget = 0;
  private double wristTarget = 0;

  public Elevator() {
    leftMotor = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless);
    //elevatorWrist = new SparkMax(Constants.Elevator.elevatorWrist, MotorType.kBrushless);

    // Default configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    leftConfig.smartCurrentLimit(40);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true); // Test: does this counteract inverting the right motor?
    leftConfig.closedLoop.outputRange(-0.1, 0.1); // 0.1 for testing
    leftConfig.closedLoop.iMaxAccum(1);
    leftConfig.closedLoop.pidf(1, 0, 0, 0);
    leftConfig.closedLoop.iZone(0);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // use PrimaryEncoder instead of AbsoluteEncoder?
    leftConfig.absoluteEncoder.positionConversionFactor(Constants.Elevator.elevatorGearboxReduction);
    leftConfig.absoluteEncoder.velocityConversionFactor(Constants.Elevator.elevatorGearboxReduction);

    // Inverts motor and encoder
    rightConfig.smartCurrentLimit(40);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.follow(Constants.Elevator.elevatorLeft, true);
    rightConfig.absoluteEncoder.positionConversionFactor(Constants.Elevator.elevatorGearboxReduction);
    rightConfig.absoluteEncoder.velocityConversionFactor(Constants.Elevator.elevatorGearboxReduction);


    // Wrist config
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //elevatorWrist.configure(
     //   wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    encoderLeft = leftMotor.getAbsoluteEncoder();
    encoderRight = rightMotor.getAbsoluteEncoder();
    //wristEncoder = elevatorWrist.getAlternateEncoder();
    pidElevator = new PIDController(0.1, 0, 0);
    pidWrist = new PIDController(0.1, 0, 0);

    leftMotorController = leftMotor.getClosedLoopController();
    rightMotorController = rightMotor.getClosedLoopController();
  }

  /**
   * Height in mm to encoder revolutions of elevator motors
   *
   * @param h
   * @return
   */
  public float elevatorHeightToEncoder(float h) {
    return h;
  }

  /**
   * Returns wrist angle, looking at motor output face, from starting config in radians
   *
   * @param r
   * @return The wrist angle in radians
   */
  public double encoderRevToWristAngle(double r) {
    r /= Constants.Elevator.wristEncoderCountsPerRevolution;
    r /= Constants.Elevator.wristGearboxReduction;
    return r * 2 * Math.PI;
  }

  public void set(double position) {
    System.out.println(position);
    //leftMotor.set(position);
    elevatorTarget = position;
    leftMotorController.setReference(elevatorTarget, ControlType.kPosition);
  }

  public void setAdditive(double additive) {
    System.out.println(elevatorTarget);
    elevatorTarget += additive;
    set(elevatorTarget);
  }

  public void setWrist(double position) {
    wristTarget = position;
    pidWrist.setSetpoint(wristTarget);
  }

  public void setWristAdditive(double additive) {
    wristTarget = pidWrist.getSetpoint() + additive;
    pidWrist.setSetpoint(wristTarget);
  }

  public double getWrist() {
    return wristEncoder.getPosition();
  }

  private int loop = 0;
  @Override
  public void periodic() {
    // leftMotor.set(pidElevator.calculate(encoderLeft.getPosition())); // Use for alternate control
    // elevatorWrist.set(pidWrist.calculate(wristEncoder.getPosition()));
    loop++;
    if (loop > 10) {
      Constants.log("Output current:" + leftMotor.getOutputCurrent());
      Constants.log("Encoder:" + encoderLeft.getPosition());
    }
  }
}
