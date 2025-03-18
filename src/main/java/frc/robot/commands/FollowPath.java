package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.List;

public class FollowPath extends Command {
  private Drivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private Timer timer;

  FollowPath(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    timer = new Timer();
    var slowConfig = new TrajectoryConfig(4, 4);
    List<Pose2d> list = new ArrayList<Pose2d>();
    list.add(new Pose2d(0, 0, new Rotation2d()));
    list.add(new Pose2d(5, 5, new Rotation2d()));
    // trajectory = TrajectoryGenerator.generateTrajectory(list, slowConfig);

    List<Waypoint> waypoints = new ArrayList<>();
    waypoints = PathPlannerPath.waypointsFromPoses(list);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(10, 6, 8, 6),
            new IdealStartingState(0, Rotation2d.kZero),
            new GoalEndState(0, Rotation2d.kZero));
    trajectory =
        path.generateTrajectory(
            new ChassisSpeeds(0, 0, 0), Rotation2d.kZero, Constants.Drivetrain.mainConfig);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    timer.start();
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    PathPlannerTrajectoryState state = trajectory.sample(timer.getTimestamp());
    Constants.log(state.pose + " " + state.fieldSpeeds.omegaRadiansPerSecond);
    drivetrain.setReference(
        state.fieldSpeeds.vxMetersPerSecond,
        state.fieldSpeeds.vyMetersPerSecond,
        state.fieldSpeeds.omegaRadiansPerSecond);
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }
}
