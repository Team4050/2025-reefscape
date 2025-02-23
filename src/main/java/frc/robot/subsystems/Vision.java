package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hazard.HazardVision;
import frc.robot.hazard.HazardXbox;

public class Vision extends SubsystemBase {
  HazardVision photonVision;
  Pose3d pose = new Pose3d();

  public Vision() {
    photonVision = new HazardVision();
  }

  int loop = 0;
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> optional = photonVision.getEstimatedGlobalPose(pose);
    if (optional.isPresent()) {
      pose = optional.get().estimatedPose;
    }
    Constants.log(pose);
    loop++;
    if (loop > 25) {
      Constants.log(pose);
      loop = 0;
    }
  }
}
