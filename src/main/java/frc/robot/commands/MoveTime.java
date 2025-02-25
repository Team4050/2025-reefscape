package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class MoveTime extends Command {
    private Drivetrain drivetrain;
    private double moveTime;
    private double xSpeed;
    private double ySpeed;
    private double angularSpeed;
    private long endTimestamp;

    public MoveTime(Drivetrain drivetrain, double xSpeed, double ySpeed, double angularSpeed, double moveTime) {
        this.drivetrain = drivetrain;
        this.moveTime = moveTime;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angularSpeed = angularSpeed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        endTimestamp = RobotController.getFPGATime() + (long)(moveTime * 1000000);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        drivetrain.set(xSpeed, ySpeed, angularSpeed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        drivetrain.set(0, 0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return endTimestamp < RobotController.getFPGATime();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        HashSet<Subsystem> s = new HashSet<Subsystem>();
        s.add(drivetrain);

        return s;
    }
}
