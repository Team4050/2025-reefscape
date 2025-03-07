package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AlignToReefTest extends Command {
  private Drivetrain drivetrain;
  private Timer timer = new Timer();
  private PIDController xController = new PIDController(0.1, 0, 0);
  private PIDController yController = new PIDController(0.1, 0, 0);
  private ProfiledPIDController omegaController = new ProfiledPIDController(0.3, 0, 0, new Constraints(0.2, 0.01));
  private Pose2d target = new Pose2d(14.58, 4.05, Rotation2d.k180deg);

  private DoubleArrayPublisher pidOut = NetworkTableInstance.getDefault().getTable("Auto command").getDoubleArrayTopic("PID").publish();

  AlignToReefTest(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(0.2);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    Pose2d p = drivetrain.getPoseEstimate();//Constants.Sensors.vision.getPose().toPose2d();
    Constants.Sensors.imu.setGyroAngleZ(p.getRotation().getDegrees());
    timer.start();
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    //Pose2d p = Constants.Sensors.vision.getPose().toPose2d();
    Pose2d p = drivetrain.getPoseEstimate();
    double vx = xController.calculate(p.getX(), target.getX());
    double vy = yController.calculate(p.getY(), target.getY());
    Rotation2d rot = target.getRotation().minus(p.getRotation());
    double omega = omegaController.calculate(rot.getRadians(), 0);
    pidOut.set(new double[] {vx, vy, omega});
    //Constants.log(vx + " " + vy + " " + -omega);
    drivetrain.set(new ChassisSpeeds(vx, vy, -omega / 2));
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
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
