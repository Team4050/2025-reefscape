package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class AutoScore extends Command {
  private Elevator elevator;
  private Claw claw;
  private Timer timeout;
  private Timer timer;
  private boolean algaeMode = false;
  private boolean cancel = false;

  public AutoScore(Elevator elevator, Claw claw) {
    this.elevator = elevator;
    this.claw = claw;
    timer = new Timer();
    timeout = new Timer();
    SmartDashboard.putString("Autoscoring status", "Inactive");
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Autoscoring status", "Waiting for arm...");
    timer.reset();
    timeout.reset();
    timeout.start();
    algaeMode = claw.algaeMode;
    super.initialize();
    if (algaeMode) {
      if (claw.hasCoral()) {
        Constants.log("Attempted to score algae with coral loaded!");
        cancel = true;
        return;
      }
    }
  }

  @Override
  public void execute() {
    if (cancel) {
      Constants.log("Command cancelled");
      return;
    }
    if (true) {
      if (elevator.isScoringL4 || elevator.isScoringL3) {
        claw.set(-Constants.Coral.coralScoreSpeed);
      } else if (algaeMode) {
        claw.set(-Constants.Coral.algaeSpeed);
      } else {
        claw.set(Constants.Coral.coralScoreSpeed);
      }
      if (!timer.isRunning() && !claw.hasCoral()) {
        Constants.log(
            "Positions within tolerances: "
                + (elevator.atElevatorReference()
                    && elevator.atShoulderReference()
                    && elevator.atWristReference()));
        SmartDashboard.putString("Autoscoring status", "Scoring...");
        timer.start();
      }
    }
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return cancel
        || (!algaeMode && timer.hasElapsed(0.75))
        || (algaeMode && timer.hasElapsed(2))
        || timeout.hasElapsed(6);
  }

  @Override
  public void end(boolean interrupted) {
    claw.set(0);
    if (timeout.hasElapsed(6)) {
      SmartDashboard.putString("Autoscoring status", "Timed out!");
      Constants.driverLog("Auto scoring request timed out!");
    } else {
      SmartDashboard.putString("Autoscoring status", "Inactive");
    }
    cancel = false;
    super.end(interrupted);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    HashSet<Subsystem> reqs = new HashSet<>();
    reqs.add(claw);
    return reqs;
  }
}
