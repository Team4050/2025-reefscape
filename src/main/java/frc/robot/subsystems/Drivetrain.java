package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  // TODO: check if kraken encoders are integrated or remote
  private Orchestra music;
  private TalonFX FL;
  private TalonFX FR;
  private TalonFX RL;
  private TalonFX RR;
  private StatusSignal<Angle> FLPos;
  private StatusSignal<Angle> FRPos;
  private StatusSignal<Angle> RLPos;
  private StatusSignal<Angle> RRPos;
  private TalonFXConfiguration leftSideConfig;
  private TalonFXConfiguration rightSideConfig;
  private MecanumDriveKinematics kinematics;

  public Drivetrain() {
    FL = new TalonFX(Constants.Drivetrain.FL);
    FR = new TalonFX(Constants.Drivetrain.FR);
    RL = new TalonFX(Constants.Drivetrain.RL);
    RR = new TalonFX(Constants.Drivetrain.RR);
    FLPos = FL.getPosition();
    FRPos = FR.getPosition();
    RLPos = RL.getPosition();
    RRPos = RR.getPosition();
    double maxOutput = 0.05; // Increase in proportion to confidence in driver skill
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput;
    leftSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
    leftSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    leftSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    leftSideConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
    leftSideConfig.Audio.AllowMusicDurDisable = true;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.MotorOutput.PeakForwardDutyCycle =
        maxOutput; // Increase in proportion to confidence in driver skill
    rightSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
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
  }

  private double moduleMetersPerSecondToKrakenRPM(double mps) {
    return Math.toDegrees(
            (mps / Constants.Drivetrain.wheelRadiusMeters)
                * Constants.Drivetrain.moduleGearReduction)
        * 60;
  }

  private double krakenRPMToMetersPerSecond(double rpm) {
    return (Math.toRadians(rpm) / 60)
        * Constants.Drivetrain.wheelRadiusMeters
        / Constants.Drivetrain.moduleGearReduction;
  }

  public double[] getEncoders() {
    // ChassisSpeeds s = kinematics.toChassisSpeeds(new
    // MecanumDriveWheelSpeeds(FL.().getValueAsDouble(), 0, 0, 0))
    double[] d = {
      FLPos.refresh().getValue().abs(edu.wpi.first.units.Units.Radians),
      FRPos.refresh().getValue().abs(edu.wpi.first.units.Units.Radians),
      RLPos.refresh().getValue().abs(edu.wpi.first.units.Units.Radians),
      RRPos.refresh().getValue().abs(edu.wpi.first.units.Units.Radians)
    };
    return d;
  }

  public void set(double xSpeed, double ySpeed, double theta) {
    MecanumDriveWheelSpeeds speeds =
        kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, theta));
    set(speeds);
  }

  public void set(MecanumDriveWheelSpeeds wheelSpeeds) {
    FL.set(moduleMetersPerSecondToKrakenRPM(wheelSpeeds.frontLeftMetersPerSecond));
    FR.set(moduleMetersPerSecondToKrakenRPM(wheelSpeeds.frontRightMetersPerSecond));
    RL.set(moduleMetersPerSecondToKrakenRPM(wheelSpeeds.rearLeftMetersPerSecond));
    RR.set(moduleMetersPerSecondToKrakenRPM(wheelSpeeds.rearRightMetersPerSecond));
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
    // Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();
  }
}
