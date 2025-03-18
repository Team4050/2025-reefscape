package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.HashSet;
import java.util.Set;

public class MoveToPositionPID extends Command {
  private Drivetrain drivetrain;
  private Timer timer = new Timer();
  private double Kp = 0.2;
  private double Ki = 0.001;
  private double Kd = 0.02; // 0.08
  private double KsX = 0.1; // 0.02
  private double KsY = 0.1;
  private double KsZ = 0;
  private PIDController xController = new PIDController(Kp, Ki, Kd);
  private PIDController yController = new PIDController(Kp, Ki, Kd);
  private ProfiledPIDController xProController =
      new ProfiledPIDController(Kp, Ki, Kd, new Constraints(1, 1));
  private ProfiledPIDController yProController =
      new ProfiledPIDController(Kp, Ki, Kd, new Constraints(1, 1));
  private ProfiledPIDController omegaController =
      new ProfiledPIDController(Kp, Ki, Kd, new Constraints(0.2, 0.2));
  private Pose2d target; // = new Pose2d(14.58, 4.05, Rotation2d.k180deg)

  private boolean cancel = false;

  private static double feedbackLimit = 0.15;
  private static double tolerance = 2 * (2.54 / 100);
  private static double iZone = 10 * (2.54 / 100);
  private static double maxIntegral = 2;

  private DoubleArrayPublisher feedbackOut =
      NetworkTableInstance.getDefault()
          .getTable("Auto command")
          .getDoubleArrayTopic("PID")
          .publish();
  private DoubleArrayPublisher feedforwardOut =
      NetworkTableInstance.getDefault()
          .getTable("Auto command")
          .getDoubleArrayTopic("Feedforward")
          .publish();

  public MoveToPositionPID(Drivetrain drivetrain, Pose2d target) {
    this.drivetrain = drivetrain;
    this.target = target;
    xController.setTolerance(tolerance);
    xController.setIZone(iZone);
    xController.setIntegratorRange(-maxIntegral, maxIntegral);

    xProController.setTolerance(tolerance);
    xProController.setIZone(iZone);
    xProController.setIntegratorRange(-maxIntegral, maxIntegral);

    yController.setTolerance(tolerance);
    yController.setIZone(iZone);
    yController.setIntegratorRange(-maxIntegral, maxIntegral);

    yProController.setTolerance(tolerance);
    yProController.setIZone(iZone);
    yProController.setIntegratorRange(-maxIntegral, maxIntegral);

    omegaController.setTolerance(Math.toRadians(2));
    omegaController.setIZone(Math.toRadians(15));
    omegaController.setIntegratorRange(-maxIntegral, maxIntegral);

    SmartDashboard.putNumber("R PID X", 0);
    SmartDashboard.putNumber("R PID Y", 0);
    SmartDashboard.putNumber("R PID Z", 0);

    SmartDashboard.putNumber("Autoalign Kp", Kp);
    SmartDashboard.putNumber("Autoalign Ki", Ki);
    SmartDashboard.putNumber("Autoalign Kd", Kd);
  }

  @Override
  public void initialize() {
    cancel = false;
    Constants.log(
        "Target position (xya): "
            + target.getX()
            + " "
            + target.getY()
            + " "
            + target.getRotation().getDegrees());
    Pose2d p = drivetrain.getPoseEstimate();
    Constants.log(
        "Initial starting position: "
            + p.getX()
            + " "
            + p.getY()
            + " "
            + p.getRotation().getDegrees());
    // Constants.Sensors.imu.setGyroAngleZ(p.getRotation().getDegrees());
    timer.restart();

    Kp = SmartDashboard.getNumber("Autoalign Kp", Kp);
    Ki = SmartDashboard.getNumber("Autoalign Ki", Ki);
    Kd = SmartDashboard.getNumber("Autoalign Kd", Kd);
    xController.setPID(Kp, Ki, Kd);
    yController.setPID(Kp, Ki, Kd);
    omegaController.setPID(Kp, Ki, Kd);
  }

  @Override
  public void execute() {
    if (cancel) return;
    // TODO Auto-generated method stub
    // Pose2d p = Constants.Sensors.vision.getPose().toPose2d();
    Pose2d p = drivetrain.getPoseEstimate();
    double vx = xController.calculate(p.getX(), target.getX());
    double vy = yController.calculate(p.getY(), target.getY());

    double vpx = xProController.calculate(p.getX(), target.getX());
    double vpy = yProController.calculate(p.getY(), target.getY());

    Rotation2d rot = target.getRotation().minus(p.getRotation());
    double omega = omegaController.calculate(rot.getRadians(), 0);

    vx = MathUtil.clamp(vx, -feedbackLimit, feedbackLimit);
    vy = MathUtil.clamp(vy, -feedbackLimit, feedbackLimit);
    vpx = MathUtil.clamp(vpx, -feedbackLimit, feedbackLimit);
    vpy = MathUtil.clamp(vpy, -feedbackLimit, feedbackLimit);
    omega = MathUtil.clamp(omega, -feedbackLimit, feedbackLimit);

    var robotRelativeSpeeds = drivetrain.toRobotRelative(new ChassisSpeeds(vx, vy, -omega));
    var feedforward =
        new ChassisSpeeds(
            Math.signum(robotRelativeSpeeds.vxMetersPerSecond) * KsX,
            Math.signum(robotRelativeSpeeds.vxMetersPerSecond) * KsY,
            Math.signum(robotRelativeSpeeds.vxMetersPerSecond) * KsZ);
    var combined = feedforward.plus(robotRelativeSpeeds);

    SmartDashboard.putNumber("R PID X", combined.vxMetersPerSecond);
    SmartDashboard.putNumber("R PID Y", combined.vyMetersPerSecond);
    SmartDashboard.putNumber("R PID Z", combined.omegaRadiansPerSecond);

    feedforwardOut.set(
        new double[] {
          feedforward.vxMetersPerSecond,
          feedforward.vyMetersPerSecond,
          feedforward.omegaRadiansPerSecond
        });
    feedbackOut.set(
        new double[] {
          robotRelativeSpeeds.vxMetersPerSecond,
          robotRelativeSpeeds.vyMetersPerSecond,
          robotRelativeSpeeds.omegaRadiansPerSecond
        });

    drivetrain.setVoltage(combined);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    if (cancel) Constants.log("Auto-align cancelled, no visible tag");
    if (drivetrain.invalidPose) Constants.log("Disabled auto-align, invalid drivetrain pose");
    return cancel
        || (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint())
        || timer.hasElapsed(5)
        || drivetrain.invalidPose;
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
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
