package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  public static SparkMax clawMotor;
  public static boolean algaeMode = false;

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
}
