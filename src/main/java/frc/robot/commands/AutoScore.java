package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class AutoScore extends Command {
  private Elevator elevator;
    private Claw claw;
    private Timer timeout;
    private Timer timer;

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
        timer.start();
        timeout.reset();
        timeout.start();
        super.initialize();
    }

    @Override
    public void execute() {
      if ((elevator.atElevatorReference() && elevator.atShoulderReference() && elevator.atWristReference()) || true) {
        if (elevator.isScoringL4 || claw.algaeMode) {
          claw.set(-1);
        } else {
          claw.set(1);
        }
        if (!timer.isRunning() && !claw.hasCoral()) {
          SmartDashboard.putString("Autoscoring status", "Scoring...");
          timer.start();
        }
      }
      super.execute();
    }

    @Override
    public boolean isFinished() {
      return (!claw.algaeMode && timer.hasElapsed(1)) || (claw.algaeMode && timer.hasElapsed(2)) || timeout.hasElapsed(6);
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
      super.end(interrupted);
    }
}
