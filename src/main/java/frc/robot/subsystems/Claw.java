package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private SparkMax clawMotor;
  private SparkMaxConfig clawConfig = new SparkMaxConfig();
  private SparkAnalogSensor breakbeam;
  public boolean algaeMode = false;
  private DoublePublisher clawSpeedPublisher =
      NetworkTableInstance.getDefault().getTable("Claw").getDoubleTopic("Speed").publish();
  private DoublePublisher breakbeamPublisher =
      NetworkTableInstance.getDefault().getTable("Claw").getDoubleTopic("Breakbeam").publish();

  public Claw() {
    clawMotor = new SparkMax(Constants.Coral.CAN, MotorType.kBrushless);

    clawConfig.idleMode(IdleMode.kBrake);
    clawConfig.inverted(true);
    clawConfig.smartCurrentLimit(Constants.Coral.currentLimit);
    clawMotor.configure(
        clawConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    breakbeam = clawMotor.getAnalog();
  }

  public boolean hasCoral() {
    return breakbeam.getVoltage() < 1.5;
  }

  /***
   * Sets the motor speed (positive moves coral out the front, negative moves coral inward)
   * @param speed
   */
  public void set(double speed) {
    // Constants.driverLog("Setting claw speed to " + speed + " algae mode " + algaeMode);
    clawSpeedPublisher.set(speed);
    clawMotor.set(speed);
  }

  /***
   * Scores the game piece indicated by the current claw mode
   */
  public void score() {}

  public void eject() {}

  /***
   * Sets the algae mode
   * @param mode
   */
  public void setAlgaeMode(boolean mode) {
    algaeMode = mode;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    breakbeamPublisher.set(breakbeam.getVoltage());
    super.periodic();
  }
}
