package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class TensionShoulder extends Command {
  private Elevator elevator;

  TensionShoulder(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    Constants.log("Tensioning - Moving shoulder motor until load detected");
  }

  @Override
  public void execute() {
    elevator.driveShoulder(0.02);
  }

  @Override
  public boolean isFinished() {
    return elevator.shoulderUnderLoad();
  }

  @Override
  public void end(boolean interrupted) {
    Constants.log("Tensioning - Shoulder load detected, stopping motor");
    elevator.driveShoulder(0);
    // TODO Auto-generated method stub
    super.end(interrupted);
  }
}
