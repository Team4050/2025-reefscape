package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hazard.HazardXbox;

public class RumbleController extends Command {
  private Timer timer = new Timer();
  private double time;
  private boolean left;
  private boolean right;
  private HazardXbox controller;

  public RumbleController(HazardXbox controller, boolean left, boolean right, double time) {
    this.controller = controller;
    this.time = time;
    this.left = left;
    this.right = right;
  }

  public RumbleController(HazardXbox controller, double time) {
    this(controller, true, true, time);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if (left) controller.rumbleLeft(1);
    if (right) controller.rumbleRight(1);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);

  }

  @Override
  public void end(boolean interrupted) {
    controller.rumble(0);
  }
}
