package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AlignToReefTrajectoryPID extends Command {
    private Drivetrain drivetrain;
  private Timer timer = new Timer();
  private double KpX = 1;
  private double KiX = 0.02;
  private double KdX = 0; //0.08

  private double KpY = 2;
  private double KiY = 0.05;
  private double KdY = 0; //0.08

  private double KpZ = 4;
  private double KiZ = 0.001;
  private double KdZ = 0; //0.08

  private double KsX = 0.2; // 0.02
  private double KsY = 0.82;
  private double KsZ = 0.6;

  private Trajectory path;
  private PIDController xController = new PIDController(KpX, KiX, KdX);
  private PIDController yController = new PIDController(KpY, KiY, KdY);
  private ProfiledPIDController omegaController = new ProfiledPIDController(KpZ, KiZ, KdZ, new Constraints(0.5, 0.2));
  private Pose2d target; // = new Pose2d(14.58, 4.05, Rotation2d.k180deg)
  private boolean right = false;
  private boolean algae = false;

  private boolean cancel = false;

  private static double feedbackLimit = 3;
  private static double tolerance = 1 * (2.54 / 100);
  private static double iZone = 10 * (2.54 / 100);
  private static double maxIntegral = 10;

  private DoubleArrayPublisher feedbackOut = NetworkTableInstance.getDefault().getTable("Auto command").getDoubleArrayTopic("PID").publish();
  private DoubleArrayPublisher feedforwardOut = NetworkTableInstance.getDefault().getTable("Auto command").getDoubleArrayTopic("Feedforward").publish();
  private DoubleArrayPublisher setpoint = NetworkTableInstance.getDefault().getTable("Auto command").getDoubleArrayTopic("Target pose").publish();

  public AlignToReefTrajectoryPID(Drivetrain drivetrain, boolean right, boolean algaeMode) {
    this.drivetrain = drivetrain;
    this.right = right;
    this.algae = algaeMode;
    xController.setTolerance(tolerance);
    xController.setIZone(iZone);
    xController.setIntegratorRange(-maxIntegral, maxIntegral);

    yController.setTolerance(tolerance);
    yController.setIZone(iZone);
    yController.setIntegratorRange(-maxIntegral, maxIntegral);

    omegaController.setTolerance(Math.toRadians(2));
    omegaController.setIZone(Math.toRadians(15));
    omegaController.setIntegratorRange(-maxIntegral, maxIntegral);

    SmartDashboard.putNumber("Autoalign X Ks", KsX);
    SmartDashboard.putNumber("Autoalign Y Ks", KsY);
    SmartDashboard.putNumber("Autoalign Z Ks", KsZ);

    SmartDashboard.putNumber("Autoalign X Kp", KpX);
    SmartDashboard.putNumber("Autoalign X Ki", KiX);
    SmartDashboard.putNumber("Autoalign X Kd", KdX);

    SmartDashboard.putNumber("Autoalign Y Kp", KpY);
    SmartDashboard.putNumber("Autoalign Y Ki", KiY);
    SmartDashboard.putNumber("Autoalign Y Kd", KdY);

    SmartDashboard.putNumber("Autoalign Z Kp", KpZ);
    SmartDashboard.putNumber("Autoalign Z Ki", KiZ);
    SmartDashboard.putNumber("Autoalign Z Kd", KdZ);
  }

  @Override
  public void initialize() {
    cancel = false;
    // TODO Auto-generated method stub
    super.initialize();
    Optional<Pose3d> tag = drivetrain.getLastSeenAprilTag();
    if (tag.isEmpty() || drivetrain.getTimeSinceLastSeenTag() > 1) {
      cancel = true;
      return;
    }
    Pose2d tagPose = tag.get().toPose2d();

    Translation2d offset;
    if (algae) {
      offset = new Translation2d(0.8, 0);
    } else {
      if (right) {
        offset = new Translation2d(0.8, 0.2 + Constants.Wrist.scoringOffsetMeters).rotateBy(tagPose.getRotation());
      } else {
        offset = new Translation2d(0.8, -0.2 + Constants.Wrist.scoringOffsetMeters).rotateBy(tagPose.getRotation());
      }
    }

    target = new Pose2d(tagPose.getTranslation().plus(offset), tagPose.getRotation().plus(Rotation2d.k180deg));
    Constants.log("Target position (xya): " + target.getX() + " " + target.getY() + " " + target.getRotation().getDegrees());
    setpoint.set(new double[] {target.getX(), target.getY(), target.getRotation().getRadians()});
    Pose2d p = drivetrain.getPoseEstimate();
    Constants.log("Initial starting position: " + p.getX()  + " " + p.getY() + " " + p.getRotation().getDegrees());
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(p);
    waypoints.add(target);
    path = TrajectoryGenerator.generateTrajectory(waypoints, null);
    //Constants.Sensors.imu.setGyroAngleZ(p.getRotation().getDegrees());
    timer.restart();

    KpX = SmartDashboard.getNumber("Autoalign X Kp", KpX);
    KiX = SmartDashboard.getNumber("Autoalign X Ki", KiX);
    KdX = SmartDashboard.getNumber("Autoalign X Kd", KdX);

    KpY = SmartDashboard.getNumber("Autoalign Y Kp", KpY);
    KiY = SmartDashboard.getNumber("Autoalign Y Ki", KiY);
    KdY = SmartDashboard.getNumber("Autoalign Y Kd", KdY);

    KpZ = SmartDashboard.getNumber("Autoalign Z Kp", KpZ);
    KiZ = SmartDashboard.getNumber("Autoalign Z Ki", KiZ);
    KdZ = SmartDashboard.getNumber("Autoalign Z Kd", KdZ);

    xController.setPID(KpX, KiX, KdX);
    yController.setPID(KpY, KiY, KdY);
    omegaController.setPID(KpZ, KiZ, KdZ);

    KsX = SmartDashboard.getNumber("Autoalign X Ks", 0);
    KsY = SmartDashboard.getNumber("Autoalign Y Ks", 0);
    KsZ = SmartDashboard.getNumber("Autoalign Z Ks", 0);

    Constants.log("PID X: " + KpX + " " + KiX + " " + KdX);
    Constants.log("PID Y: " + KpY + " " + KiY + " " + KdY);
    Constants.log("PID Z: " + KpZ + " " + KiZ + " " + KdZ);
    Constants.log("Kstatic: " + KsX + " " + KsY + " " + KsZ);
  }

  @Override
  public void execute() {
    if (cancel) return;
    // TODO Auto-generated method stub
    //Pose2d p = Constants.Sensors.vision.getPose().toPose2d();
    Pose2d p = drivetrain.getPoseEstimate();
    double vx = xController.calculate(p.getX(), target.getX());
    if (xController.atSetpoint()) {
      Constants.log("At X setpoint");
      vx = 0;
    }
    double vy = yController.calculate(p.getY(), target.getY());
    if (yController.atSetpoint()) {
      Constants.log("At Y setpoint");
      vy = 0;
    }

    Rotation2d rot = p.getRotation().minus(target.getRotation());
    double omega = omegaController.calculate(rot.getRadians(), 0);
    if (omegaController.atSetpoint()) {
      Constants.log("At Z setpoint");
      omega = 0;
    }

    vx = MathUtil.clamp(vx, -feedbackLimit, feedbackLimit);
    vy = MathUtil.clamp(vy, -feedbackLimit, feedbackLimit);
    omega = MathUtil.clamp(omega, -feedbackLimit, feedbackLimit);

    var feedback = drivetrain.toRobotRelative(new ChassisSpeeds(vx, vy, omega));
    var feedforward = new ChassisSpeeds(Math.signum(feedback.vxMetersPerSecond) * KsX, Math.signum(feedback.vyMetersPerSecond) * KsY, Math.signum(feedback.omegaRadiansPerSecond) * KsZ);
    var combined = feedforward.plus(feedback);

    SmartDashboard.putNumber("R PID X", combined.vxMetersPerSecond);
    SmartDashboard.putNumber("R PID Y", combined.vyMetersPerSecond);
    SmartDashboard.putNumber("R PID Z", combined.omegaRadiansPerSecond);
    feedforwardOut.set(new double[] {feedforward.vxMetersPerSecond, feedforward.vyMetersPerSecond, feedforward.omegaRadiansPerSecond});
    feedbackOut.set(new double[] {feedback.vxMetersPerSecond, feedback.vyMetersPerSecond, feedback.omegaRadiansPerSecond});
    drivetrain.setVoltage(combined);
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    if (cancel) Constants.log("Auto-align cancelled, no visible tag");
    if (drivetrain.invalidPose) Constants.log("Disabled auto-align, invalid drivetrain pose");
    return cancel || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint()) || timer.hasElapsed(5) || drivetrain.invalidPose;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) Constants.log("Auto-align interrupted");
    drivetrain.set(0, 0, 0);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    Set<Subsystem> reqs = new HashSet<>();
    reqs.add(drivetrain);
    return reqs;
  }
}
