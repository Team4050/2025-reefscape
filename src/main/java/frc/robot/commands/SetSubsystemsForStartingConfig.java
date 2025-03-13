package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class SetSubsystemsForStartingConfig extends Command {
    private Elevator elevator;
    private Claw claw;

    public SetSubsystemsForStartingConfig(Elevator elevator, Claw claw) {
        this.elevator = elevator;
        this.claw = claw;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> reqs = new HashSet<Subsystem>();
        reqs.add(elevator);
        reqs.add(claw);
        return reqs;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        elevator.set(0);
        elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
        elevator.setWrist(Constants.Wrist.startingRotationRadians);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
