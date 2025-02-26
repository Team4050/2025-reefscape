package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardVision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class Vision {
  HazardVision photonVision;
  Pose3d pose = new Pose3d();

  public Vision() {
    photonVision = new HazardVision();
  }

  int loop = 0;

  public void periodic() {
    //photonVision.addHeadingData(Constants.Sensors.getImuRotation3d());
    Optional<EstimatedRobotPose> optional = photonVision.getEstimatedGlobalPose();
    if (optional.isPresent()) {
      pose = optional.get().estimatedPose;
    }
    //Constants.log(pose);
    loop++;
    if (loop > 25) {
      Constants.log(pose.getRotation().getZ());
      loop = 0;
    }
  }
}
