package frc.robot.hazard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HazardDashboardNumber {
  private String name;

  HazardDashboardNumber(String name, double value) {
    this.name = name;
    SmartDashboard.putNumber(name, value);
  }

  public double get() {
    //SmartDashboard.getNum
    return SmartDashboard.getNumber(name, 0);
  }

  public boolean set(double value) {
    return SmartDashboard.putNumber(name, value);
  }
}
