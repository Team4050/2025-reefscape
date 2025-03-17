package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AutoLoad extends Command {
    private Elevator elevator;
    private Timer timeout;
    private Timer timer;
    private boolean sensed = false;
    private boolean algaeMode = false;
    private boolean cancel = false;

    public AutoLoad(Elevator elevator) {
      this.elevator = elevator;
        timer = new Timer();
        timeout = new Timer();
        SmartDashboard.putString("Autoloading status", "Inactive");
    }

    @Override
    public void initialize() {
      SmartDashboard.putString("Autoloading status", "Feeding");
      cancel = false;
      timeout.restart();
        algaeMode = elevator.algaeMode;
        if (algaeMode) {
          SmartDashboard.putString("Autoloading status", "Feeding algae");
          timer.start();
        } else {
          if (elevator.hasCoral()) {
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
        elevator.setClaw(0.1);
        if (!sensed && elevator.hasCoral()) {
          Constants.log("Timer started");
          timer.restart();
          sensed = true;
        }
      } else {
        elevator.setClaw(-0.3);
      }
    }

    @Override
    public boolean isFinished() {
      Constants.log(timer.get());
      return cancel || (!algaeMode && sensed && timer.hasElapsed(0.28)) || (algaeMode && timer.hasElapsed(2));
    }

    @Override
    public void end(boolean interrupted) {
      sensed = false;
      timer.stop();
      elevator.setClaw(0);
      if (timeout.hasElapsed(6)) {
        SmartDashboard.putString("Autoloading status", "Timed out!");
        Constants.driverLog("Auto loading request timed out!");
      } else {
        SmartDashboard.putString("Autoloading status", "Inactive");
      }
      super.end(interrupted);
    }
}
