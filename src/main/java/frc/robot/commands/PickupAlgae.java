package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class PickupAlgae extends Command {
  private Timer timer = new Timer();
  private Claw claw;

  PickupAlgae(Claw claw) {
    this.claw = claw;
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  @Override
  public void execute() {
    claw.set(0.3);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return timer.hasElapsed(1);
  }

  @Override
  public void end(boolean interrupted) {
    claw.set(0);
    super.end(interrupted);
  }
}