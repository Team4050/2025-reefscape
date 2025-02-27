package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ReefAlign extends Command {
    private Drivetrain drivetrain;
    private Vision vision;
    private HolonomicDriveController holonomicDriveController;

    ReefAlign() {
        holonomicDriveController = new HolonomicDriveController(
            new PIDController(0.1, 0, 0), 
            new PIDController(0.1, 0, 0),
            new ProfiledPIDController(0, 0, 0, 
            new Constraints(1, 3)));
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = holonomicDriveController.calculate(null, null, null);
        //drivetrain.set(null);
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}