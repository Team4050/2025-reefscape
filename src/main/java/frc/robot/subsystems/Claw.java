package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private SparkMax clawMotor;
  public boolean algaeMode = false;

  public Claw() {
    //clawMotor = new SparkMax(Constants.Coral.CAN, MotorType.kBrushless);
  }

  /***
   * Sets the motor speed
   * @param speed
   */
  public void set(double speed) {
    //clawMotor.set(speed);
  }

  /***
   * Sets the algae mode
   * @param mode
   */
  public void setAlgaeMode(boolean mode) {
    algaeMode = mode;
  }
}
