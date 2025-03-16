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

    public AutoLoad(Elevator elevator, Claw claw) {
      this.elevator = elevator;
        this.claw = claw;
        timer = new Timer();
        timeout = new Timer();
        SmartDashboard.putString("Autoloading status", "Inactive");
    }

    @Override
    public void initialize() {
      SmartDashboard.putString("Autoloading status", "Waiting for arm...");
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
          claw.set(-0.1);
        } else {
          claw.set(0.1);
        }
        if (!timer.isRunning() && !claw.hasCoral()) {
          SmartDashboard.putString("Autoloading status", "Scoring...");
          timer.start();
        }
      }
      super.execute();
    }

    @Override
    public boolean isFinished() {
      return (!claw.algaeMode && timer.hasElapsed(0.75)) || (claw.algaeMode && timer.hasElapsed(2)) || timeout.hasElapsed(6);
    }

    @Override
    public void end(boolean interrupted) {
      claw.set(0);
      if (timeout.hasElapsed(6)) {
        SmartDashboard.putString("Autoloading status", "Timed out!");
        Constants.driverLog("Auto loading request timed out!");
      } else {
        SmartDashboard.putString("Autoloading status", "Inactive");
      }
      super.end(interrupted);
    }
}
