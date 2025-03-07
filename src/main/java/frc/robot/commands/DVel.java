package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DVel extends Command {
  private Drivetrain drivetrain;
  private Timer timer = new Timer();

  DVel(Drivetrain d) {
    drivetrain = d;
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    drivetrain.setReference(1, 0, 0, 0, 0, 0);
    timer.start();
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    drivetrain.followReference();
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return timer.hasElapsed(1);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    drivetrain.set(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    Set<Subsystem> s = new HashSet<>();
    s.add(drivetrain);
    return super.getRequirements();
  }
}
