package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private TalonFX FL;
  private TalonFX FR;
  private TalonFX RL;
  private TalonFX RR;
  private TalonFXConfiguration leftSideConfig;
  private TalonFXConfiguration rightSideConfig;
  private MecanumDriveKinematics kinematics;

  public Drivetrain() {
    FL = new TalonFX(Constants.Drivetrain.FL);
    FR = new TalonFX(Constants.Drivetrain.FR);
    RL = new TalonFX(Constants.Drivetrain.RL);
    RR = new TalonFX(Constants.Drivetrain.RR);
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    FL.getConfigurator().apply(leftSideConfig);
    RL.getConfigurator().apply(leftSideConfig);

    FR.getConfigurator().apply(rightSideConfig);
    RR.getConfigurator().apply(rightSideConfig);

    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(0.5588, 0.5588),
            new Translation2d(0.5588, -0.5588),
            new Translation2d(-0.5588, 0.5588),
            new Translation2d(-0.5588, -0.5588));
  }

  private double moduleMetersPerSecondToKrakenRPM(double mps) {
    double speedRadians =
        (mps / Constants.Drivetrain.wheelRadiusMeters) * Constants.Drivetrain.moduleGearReduction;
    return Math.toDegrees(speedRadians) * 60; // Deg/m
  }

  public void set(double xSpeed, double ySpeed, double theta) {
    MecanumDriveWheelSpeeds speeds =
        kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, theta));
    set(speeds);
  }

  public void set(MecanumDriveWheelSpeeds wheelSpeeds) {}

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
  }
}
