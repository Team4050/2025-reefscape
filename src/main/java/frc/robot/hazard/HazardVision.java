/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.hazard;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Copied from 2024 code */
public class HazardVision {
  public PhotonCamera chassis;
  // public PhotonCamera claw;
  private PhotonPoseEstimator poseEstimator;

  public HazardVision() {
    chassis = new PhotonCamera("Limelight");

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.CLOSEST_TO_LAST_POSE,
            new Transform3d(0, 0, 0, new Rotation3d()));
    /*catch (UncheckedIOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      poseEstimator = null;
    }*/
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (poseEstimator == null) {
      return Optional.empty();
    }
    Optional<EstimatedRobotPose> latest = Optional.empty();
    for (PhotonPipelineResult r : chassis.getAllUnreadResults()) {
      latest = poseEstimator.update(r);
    }
    return latest;
  }

  public void setLastPose(Pose3d pose) {
    poseEstimator.setLastPose(pose);
  }

  /***
   * Add real-time heading data to the pose estimator
   * @param heading
   */
  public void addHeadingData(Rotation3d heading) {
    poseEstimator.addHeadingData((double)RobotController.getFPGATime(), heading);
  }
}
