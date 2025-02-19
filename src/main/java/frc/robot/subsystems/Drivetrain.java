package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //TODO: check if kraken encoders are integrated or remote
  private Orchestra music;
  private TalonFX FL;
  private TalonFX FR;
  private TalonFX RL;
  private TalonFX RR;
  private TalonFXConfiguration leftSideConfig;
  private TalonFXConfiguration rightSideConfig;
  private MecanumDriveKinematics kinematics;
  private ADIS16470_IMU imu;

  public Drivetrain() {
    FL = new TalonFX(Constants.Drivetrain.FL);
    FR = new TalonFX(Constants.Drivetrain.FR);
    RL = new TalonFX(Constants.Drivetrain.RL);
    RR = new TalonFX(Constants.Drivetrain.RR);
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    leftSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    leftSideConfig.Audio.AllowMusicDurDisable = true;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    rightSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    rightSideConfig.Audio.AllowMusicDurDisable = true;

    music.addInstrument(FL, 0);
    music.addInstrument(FR, 1);
    music.addInstrument(RL, 2);
    music.addInstrument(RR, 3);
    music.loadMusic("E1M1.chrp");

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

    imu = new ADIS16470_IMU(IMUAxis.kZ, IMUAxis.kY, IMUAxis.kX);
    imu.configCalTime(ADIS16470_IMU.CalibrationTime._4s);
    imu.calibrate();
  }

  private double moduleMetersPerSecondToKrakenRPM(double mps) {
    return Math.toDegrees((mps / Constants.Drivetrain.wheelRadiusMeters) * Constants.Drivetrain.moduleGearReduction) * 60;
  }

  private double krakenRPMToMetersPerSecond(double rpm) {
    return (Math.toRadians(rpm) / 60) * Constants.Drivetrain.wheelRadiusMeters / Constants.Drivetrain.moduleGearReduction;
  }

  public ChassisSpeeds getEncoders() {
    //ChassisSpeeds s = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(FL.().getValueAsDouble(), 0, 0, 0))
    return new ChassisSpeeds();
  }

  public void set(double xSpeed, double ySpeed, double theta) {
    MecanumDriveWheelSpeeds speeds =
        kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, theta));
    set(speeds);
  }

  public void set(MecanumDriveWheelSpeeds wheelSpeeds) {

  }

  public void play() {
    music.play();
  }

  public void pause() {
    music.pause();
  }

  public void stop() {
    music.stop();
  }

  @Override
  public void periodic() {
    //Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();
  }
}
