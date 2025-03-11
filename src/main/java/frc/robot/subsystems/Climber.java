package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardSparkMax;

public class Climber extends SubsystemBase {
  private HazardSparkMax motor;
  private double targetPosition = 0;

  public Climber() {
    SparkMaxConfig config = new SparkMaxConfig();
    motor = new HazardSparkMax(Constants.Climber.climber, MotorType.kBrushless, config, true, "Climber");
  }

  /***
   * Set the target position of the climber
   * @param position 0 is vertical, positive is inwards rowards the robot, negative is outwards
   */
  public void set(double position) {
    targetPosition = position;
    motor.set(0); //TODO: follow position
  }

  public void setAdditive(double additive) {
    set(targetPosition + additive);
  }
}
