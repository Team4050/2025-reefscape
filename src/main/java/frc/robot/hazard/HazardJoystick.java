package frc.robot.hazard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class HazardJoystick extends CommandJoystick {

  private float deadzone = 0.05f;
  private Joystick hid;

  /**
   * Initializes a new joystick with the specified port and deadzone
   *
   * @param port USB port of the joystick
   * @param deadzone Deadzone (0 - 1)
   */
  public HazardJoystick(int port, float deadzone) {
    this(port);

    this.deadzone = deadzone;
    this.hid = super.getHID();
  }

  /**
   * Initializes a new joystick with the specified port
   *
   * @param port USB port of the joystick
   */
  public HazardJoystick(int port) {
    super(port);

    this.hid = super.getHID();
  }

  /**
   * Rumble the joystick
   *
   * @param amount Value to rumble (0 - 1)
   */
  public void rumble(float amount) {
    this.rumbleLeft(amount);
    this.rumbleRight(amount);
  }

  /**
   * Rumble the left side of the joystick
   *
   * @param amount Value to rumble (0 - 1)
   */
  public void rumbleLeft(float amount) {
    this.hid.setRumble(GenericHID.RumbleType.kLeftRumble, amount);
  }

  /**
   * Rumble the right side of the joystick
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
  public double getX() {
    return this.deadzone(this.deadzone, super.getX());
  }

  public double getX(float deadzone) {
    return this.deadzone(deadzone, super.getX());
  }

  @Override
  public double getY() {
    return this.deadzone(this.deadzone, super.getY());
  }

  public double getY(float deadzone) {
    return this.deadzone(deadzone, super.getY());
  }

  @Override
  public double getZ() {
    return this.deadzone(this.deadzone, super.getZ());
  }

  public double getZ(float deadzone) {
    return this.deadzone(deadzone, super.getZ());
  }
}
