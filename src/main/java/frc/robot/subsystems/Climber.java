package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hazard.HazardSparkMax;

public class Climber extends SubsystemBase {
  private HazardSparkMax motor;
  private double targetPosition = 0;

  public Climber() {
    
  }

  /***
   * Set the target position of the climber
   * @param position 0 is vertical, positive is inwards rowards the robot, negative is outwards
   */
  public void set(double position) {
    targetPosition = position;
  }

  public void setAdditive(double additive) {
    set(targetPosition + additive);
  }
}
