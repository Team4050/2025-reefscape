package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class AutoLoad extends Command {
  private Elevator elevator;
  private Claw claw;
  private Timer timeout;
  private Timer timer;
  private boolean sensed = false;
  private boolean algaeMode = false;
  private boolean cancel = false;

  public AutoLoad(Elevator elevator, Claw claw) {
    this.elevator = elevator;
    this.claw = claw;
    timer = new Timer();
    timeout = new Timer();
    SmartDashboard.putString("Autoloading status", "Inactive");
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Autoloading status", "Feeding");
    cancel = false;
    timeout.restart();
    timer.stop();
    timer.reset();
    algaeMode = claw.algaeMode;
    if (algaeMode) {
      SmartDashboard.putString("Autoloading status", "Feeding algae");
      timer.start();
    } else {
      if (claw.hasCoral()) {
        SmartDashboard.putString("Autoloading status", "Already loaded");
        cancel = true;
      }
    }
    super.initialize();
  }

  @Override
  public void execute() {
    if (cancel) return;
    if (!algaeMode) {
      claw.set(Constants.Coral.coralSpeed);
      if (!sensed && claw.hasCoral()) {
        Constants.log("Timer started");
        timer.restart();
        sensed = true;
      }
    } else {
      claw.set(Constants.Coral.algaeSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    // Constants.log(timer.get());
    return cancel
        || (!algaeMode && sensed && timer.hasElapsed(0.3)) // 0.25 sec org
        || (algaeMode && timer.hasElapsed(4)); // 2 sec org
  }

  @Override
  public void end(boolean interrupted) {
    sensed = false;
    timer.stop();
    claw.set(0);
    if (cancel) {
      Constants.log("Autoloading cancelled: coral detected in claw");
      SmartDashboard.putString("Autoloading status", "Cancelled");
    }
    if (timeout.hasElapsed(6)) {
      SmartDashboard.putString("Autoloading status", "Inactive");
      Constants.driverLog("You have bad timing");
    } else {
      SmartDashboard.putString("Autoloading status", "Inactive");
    }
    super.end(interrupted);
  }
}
