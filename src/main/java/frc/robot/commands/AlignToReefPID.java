package frc.robot.commands;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AlignToReefPID extends Command {
  private Drivetrain drivetrain;
  private Timer timer = new Timer();
  private double Kp = 0.3;
  private double Ki = 0.05;
  private double Kd = 0.03;
  private double Ks = 0.03;
  private PIDController xController = new PIDController(Kp, Ki, Kd);
  private PIDController yController = new PIDController(Kp, Ki, Kd);
  private ProfiledPIDController xProController = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(0.25, 0.4));
  private ProfiledPIDController yProController = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(0.25, 0.4));
  private ProfiledPIDController omegaController = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(0.2, 0.2));
  private Pose2d target = new Pose2d(14.58, 4.05, Rotation2d.k180deg);
  private boolean right = false;
  private boolean algae = false;

  private boolean cancel = false;

  private static double feedbackLimit = 0.15;
  private static double tolerance = 2 * (2.54 / 100);
  private static double maxIntegral = 2;

  private DoubleArrayPublisher pidOut = NetworkTableInstance.getDefault().getTable("Auto command").getDoubleArrayTopic("PID").publish();

  public AlignToReefPID(Drivetrain drivetrain, boolean right, boolean algaeMode) {
    this.drivetrain = drivetrain;
    this.right = right;
    this.algae = algaeMode;
    xController.setTolerance(tolerance);
    xController.setIntegratorRange(-maxIntegral, maxIntegral);

    xProController.setTolerance(tolerance);
    xProController.setIntegratorRange(-maxIntegral, maxIntegral);

    yController.setTolerance(tolerance);
    yController.setIntegratorRange(-maxIntegral, maxIntegral);

    yProController.setTolerance(tolerance);
    yProController.setIntegratorRange(-maxIntegral, maxIntegral);

    omegaController.setTolerance(Math.toRadians(2));
    omegaController.setIntegratorRange(-maxIntegral, maxIntegral);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    Optional<Pose3d> tag = drivetrain.getLastSeenAprilTag();
    if (tag.isEmpty()) {
      cancel = true;
      return;
    }
    Pose2d tagPose = tag.get().toPose2d();

    Translation2d offset;
    if (algae) {
      offset = new Translation2d(0.6, 0);
    } else {
      if (right) {
        offset = new Translation2d(0.6, 0.2).rotateBy(tagPose.getRotation());
      } else {
        offset = new Translation2d(0.6, -0.2).rotateBy(tagPose.getRotation());
      }
    }

    target = new Pose2d(tagPose.getTranslation().plus(offset), tagPose.getRotation().plus(Rotation2d.k180deg));
    Constants.log("Target position (xya): " + target.getX() + " " + target.getY() + " " + target.getRotation().getDegrees());
    Pose2d p = drivetrain.getPoseEstimate();
    Constants.log("Initial starting position: " + p.getX()  + " " + p.getY() + " " + p.getRotation().getDegrees());
    //Constants.Sensors.imu.setGyroAngleZ(p.getRotation().getDegrees());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (cancel) return;
    // TODO Auto-generated method stub
    //Pose2d p = Constants.Sensors.vision.getPose().toPose2d();
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

    double KstaticX = Math.signum(vpx) * Ks;
    double KstaticY = Math.signum(vpy) * Ks;
    double KstaticZ = Math.signum(omega) * Ks;

    pidOut.set(new double[] {vpx + KstaticX, vpy + KstaticY, omega + KstaticZ});
    drivetrain.setFieldRelative(new ChassisSpeeds(vpx + KstaticX, vpy + KstaticY, -(omega + KstaticZ)));
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
