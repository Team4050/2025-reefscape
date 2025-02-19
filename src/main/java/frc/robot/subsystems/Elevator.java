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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  //private SparkMax elevatorWrist;
  private SparkClosedLoopController leftMotorController;
  private SparkClosedLoopController rightMotorController;
  private AbsoluteEncoder encoderLeft;
  private AbsoluteEncoder encoderRight;
  private RelativeEncoder wristEncoder;
  private PIDController pidLeft;
  private PIDController pidRight;
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

    leftConfig.closedLoop.pid(1, 0, 0);
    leftConfig.closedLoop.velocityFF(0);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Inverts motor and encoder
    rightConfig.follow(Constants.Elevator.elevatorLeft, true);

    // Wrist config
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    //elevatorWrist.configure(
     //   wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    encoderLeft = leftMotor.getAbsoluteEncoder();
    encoderRight = rightMotor.getAbsoluteEncoder();
    //wristEncoder = elevatorWrist.getAlternateEncoder();
    pidWrist = new PIDController(0.1, 0, 0);

    leftMotorController = leftMotor.getClosedLoopController();
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
    leftMotor.set(position);
    //elevatorTarget = position;
    //leftMotorController.setReference(elevatorTarget, ControlType.kPosition);
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

  @Override
  public void periodic() {
    //elevatorWrist.set(pidWrist.calculate(wristEncoder.getPosition()));
    System.out.println("Output current:" + leftMotor.getOutputCurrent());
    System.out.println("Encoder:" + encoderLeft.getPosition());
  }
}
