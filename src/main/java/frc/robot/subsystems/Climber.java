package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardSparkMax;

public class Climber extends SubsystemBase {
  private HazardSparkMax motor;
  private double targetPosition = 0;

  public Climber() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(Constants.Climber.currentLimit);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pidf(0.2, 0, 0, 0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    config.closedLoop.maxMotion.maxAcceleration(11.5);
    config.closedLoop.maxMotion.maxVelocity(11.5);
    config.encoder.velocityConversionFactor(Constants.Climber.climberGearReduction);
    config.encoder.positionConversionFactor(Constants.Climber.climberGearReduction);
    motor = new HazardSparkMax(Constants.Climber.climber, MotorType.kBrushless, config, true, "Climber");
  }

  /***
   * Set the target position of the climber
   * @param position 0 is vertical, positive is inwards rowards the robot, negative is outwards
   */
  public void set(double position) {
    targetPosition = MathUtil.clamp(position, Constants.Climber.deployedPositionRotations, Constants.Climber.climbedPositionRotations);
    //Constants.log(targetPosition);
    motor.setControl(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void setAdditive(double additive) {
    set(targetPosition + additive * 0.02);
  }

  public boolean ratcheted() {
    return motor.getPosition() > Constants.Climber.climbedPositionRotations;
  }

  @Override
  public void periodic() {
    if (ratcheted()) {
      motor.set(0);
    } else {
      var dif = targetPosition - motor.getPosition();
      if (Math.abs(dif) > 0.01) {
        if (targetPosition < motor.getPosition()) {
          motor.set(-0.6);
        } else if (targetPosition > motor.getPosition()) {
          motor.set(0.6);
        }
      } else {
        motor.set(0);
      }
    }

    motor.publishToNetworkTables();
    // TODO Auto-generated method stub
    super.periodic();
  }
}
