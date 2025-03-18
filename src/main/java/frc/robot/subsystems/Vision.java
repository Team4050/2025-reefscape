package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.hazard.HazardVision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision {
  HazardVision photonVision;
  Pose3d pose = new Pose3d();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
  DoubleArrayPublisher posePublisher = table.getDoubleArrayTopic("Robot pose meters").publish();

  public enum Pipelines {
    FAST,
    PRECISE
  };

  public Vision() {
    Constants.log(Pipelines.FAST.ordinal());
    Constants.log(Pipelines.PRECISE.ordinal());
    photonVision = new HazardVision();
  }

  public void setRobotPose(Pose3d pose) {
    photonVision.setLastPose(pose);
  }

  public void setPipeline(Pipelines pipeline) {
    photonVision.setPipeline(pipeline.ordinal());
  }

  public void setPipelineIndex(int index) {
    photonVision.setPipeline(index);
  }

  public Pose3d getPose() {
    return pose;
  }

  int loop = 0;

  public void periodic() {
    // photonVision.addHeadingData(Constants.Sensors.getImuRotation3d());
    photonVision.addHeadingData(Constants.Sensors.getImuRotation3d());
    Optional<EstimatedRobotPose> optional = photonVision.getEstimatedGlobalPose();
    if (optional.isPresent()) {
      pose = optional.get().estimatedPose;
    }

    posePublisher.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getZ()});

    loop++;
    if (loop > 25) {
      Constants.log("Pose Z:" + pose.getRotation().getZ());
      loop = 0;
    }
  }
}
