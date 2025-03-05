package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  public static SparkMax clawMotor;

  public Claw() {
    clawMotor = new SparkMax(Constants.Coral.CAN, MotorType.kBrushless);
  }

  /***
   * Sets the motor speed
   * @param speed
   */
  public void set(float speed) {
    clawMotor.set(speed);
  }
}
