package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.Optional;

public class AlignToReefTrajectory extends Command {
  private Drivetrain drivetrain;
  private HolonomicDriveController holonomicDriveController;
  private Translation2d reefPolePosition;
  private Rotation2d reefNormal;
  private PathPlannerTrajectory trajectory;
  private Timer timer = new Timer();
  private boolean right = false;
  private boolean cancel = false;

  /***
   * Aligns the robot to score on the reef. Defaults to left pole.
   * @param right If the command will align to the right pole
   */
  public AlignToReefTrajectory(Drivetrain drivetrain, boolean right) {
    this.drivetrain = drivetrain;
    this.right = right;
    holonomicDriveController =
        new HolonomicDriveController(
            new PIDController(0.1, 0, 0),
            new PIDController(0.1, 0, 0),
            new ProfiledPIDController(0.3, 0, 0, new Constraints(1, 3)));
  }

  public void regeneratePath() {
    Pose2d initial = drivetrain.getPoseEstimate();
    var waypoints = new ArrayList<PathPoint>();
    waypoints.add(new PathPoint(initial.getTranslation()));
    waypoints.add(new PathPoint(reefPolePosition));
    trajectory =
        PathPlannerPath.fromPathPoints(
                waypoints, new PathConstraints(3, 1, 3, 1), new GoalEndState(0, reefNormal))
            .generateTrajectory(
                drivetrain.getFieldRelativeSpeedsMPS(),
                initial.getRotation(),
                Constants.Drivetrain.mainConfig);
  }

  @Override
  public void initialize() {
    Pose2d initial = drivetrain.getPoseEstimate();
    Optional<Pose3d> tag = drivetrain.getLastSeenAprilTag();
    if (tag.isEmpty()) {
      cancel = true;
      return;
    }
    Pose2d tagPosition = tag.get().toPose2d();
    Translation2d offset = new Translation2d(0.5, 0.3).rotateBy(tagPosition.getRotation());
    if (right) {
      offset = new Translation2d(0.5, -0.3).rotateBy(tagPosition.getRotation());
    }
    reefPolePosition = tagPosition.getTranslation().plus(offset);
    reefNormal = tagPosition.getRotation().plus(Rotation2d.k180deg);
    Constants.log(
        "Target position (xya): "
            + reefPolePosition.getX()
            + " "
            + reefPolePosition.getY()
            + " "
            + reefPolePosition.getAngle().getDegrees());
    super.initialize();
    timer.start();
  }

  @Override
  public void execute() {
    var state = trajectory.sample(timer.get());
    drivetrain.driveFieldRelative(state.pose, state.linearVelocity);
    // drivetrain.set(null);
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
    return holonomicDriveController.atReference() || timer.hasElapsed(15);
  }
}
