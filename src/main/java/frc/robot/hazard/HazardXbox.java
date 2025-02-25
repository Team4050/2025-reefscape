package frc.robot.hazard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class HazardXbox extends CommandXboxController {

  private float deadzone = 0.08f;
  private XboxController hid;

  /**
   * Initializes a new Xbox controller with the specified port and deadzone
   *
   * @param port USB port of the controller
   * @param deadzone Deadzone (0 - 1)
   */
  public HazardXbox(int port, float deadzone) {
    this(port);

    this.deadzone = deadzone;
    this.hid = super.getHID();
  }

  /**
   * Initializes a new Xbox controller with the specified port
   *
   * @param port USB port of the controller
   */
  public HazardXbox(int port) {
    super(port);

    this.hid = super.getHID();
  }

  /**
   * Rumble the controller
   *
   * @param amount Value to rumble (0 - 1)
   */
  public void rumble(float amount) {
    this.rumbleLeft(amount);
    this.rumbleRight(amount);
  }

  /**
   * Rumble the left side of the controller
   *
   * @param amount Value to rumble (0 - 1)
   */
  public void rumbleLeft(float amount) {
    this.hid.setRumble(GenericHID.RumbleType.kLeftRumble, amount);
  }

  /**
   * Rumble the right side of the controller
   *
   * @param amount Value to rumble (0 - 1)
   */
  public void rumbleRight(float amount) {
    this.hid.setRumble(GenericHID.RumbleType.kRightRumble, amount);
  }

  /* The below code consists of overrides and new methods to apply deadzone */

  /**
   * Returns the value of the axis after accounting for the configured deadzone
   *
   * @param v The value of the axis
   * @return The value of the axis after accounting for deadzone
   */
  private double deadzone(float deadzone, double v) {
    return Math.abs(v) > deadzone ? v : 0;
  }

  @Override
  public double getLeftX() {
    return this.deadzone(this.deadzone, super.getLeftX());
  }

  public double getLeftX(float deadzone) {
    return this.deadzone(deadzone, super.getLeftX());
  }

  @Override
  public double getLeftY() {
    return this.deadzone(this.deadzone, super.getLeftY());
  }

  public double getLeftY(float deadzone) {
    return this.deadzone(deadzone, super.getLeftY());
  }

  @Override
  public double getRightX() {
    return this.deadzone(this.deadzone, super.getRightX());
  }

  public double getRightX(float deadzone) {
    return this.deadzone(deadzone, super.getRightX());
  }

  @Override
  public double getRightY() {
    return this.deadzone(this.deadzone, super.getRightY());
  }

  public double getRightY(float deadzone) {
    return this.deadzone(deadzone, super.getRightY());
  }
}
