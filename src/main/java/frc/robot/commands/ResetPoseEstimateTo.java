package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ResetPoseEstimateTo extends Command {
  private Drivetrain drivetrain;
  private Pose2d pose;

  public ResetPoseEstimateTo(Drivetrain drivetrain, Pose2d pose) {
    this.drivetrain = drivetrain;
    this.pose = pose;
  }

  @Override
  public void execute() {
    Constants.Sensors.setIMUGyroYaw(pose.getRotation().getDegrees());
    drivetrain.resetPoseEstimate(pose);
    // TODO Auto-generated method stub
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
