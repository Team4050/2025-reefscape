package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMax elevatorWrist;
  private AbsoluteEncoder encoderLeft;
  private AbsoluteEncoder encoderRight;
  private AnalogEncoder wristEncoder;
  private PIDController pidLeft;
  private PIDController pidRight;
  private PIDController pidWrist;

  public Elevator() {
    // Default configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    // Inverts motor and encoder
    leftConfig.inverted(true);

    leftMotor = new SparkMax(Constants.Elevator.elevatorLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.elevatorRight, MotorType.kBrushless);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    encoderLeft = leftMotor.getAbsoluteEncoder();
    encoderRight = rightMotor.getAbsoluteEncoder();
    wristEncoder = new AnalogEncoder(0);
    pidLeft = new PIDController(0.1, 0, 0);
    pidRight = new PIDController(0.1, 0, 0);
    pidWrist = new PIDController(0.1, 0, 0);
  }

  // Height in mm to encoder revolutions of elevator motors
  private float convertHeightToEncoder(float h) {
    return h;
  }

  public void set(double position) {
    pidLeft.setSetpoint(position);
    pidRight.setSetpoint(position);
  }

  public void setAdditive(double additive) {
    double position = pidLeft.getSetpoint() + additive;
    set(position);
  }

  public void setWrist(double additive) {
    double position = pidWrist.getSetpoint() + additive;
    pidWrist.setSetpoint(position);
  }

  @Override
  public void periodic() {
    leftMotor.set(pidLeft.calculate(encoderLeft.getPosition()));
    rightMotor.set(pidRight.calculate(encoderRight.getPosition()));
    elevatorWrist.set(pidWrist.calculate(wristEncoder.get()));

    System.out.println(encoderLeft.getPosition());
  }
}
