// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ReturnAutonomous;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrainSubsystem = new Drivetrain(true, 0, new Pose2d(15, 4, Rotation2d.k180deg));
  private final Elevator elevatorSubsystem = new Elevator();
  //private final Claw clawSubsystem = new Claw();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox m_driverController =
      new HazardXbox(OperatorConstants.kDriverControllerPort);
  private final HazardXbox m_secondaryController = 
      new HazardXbox(OperatorConstants.kSecondaryControllerPort);

  private NetworkTableInstance netTables = NetworkTableInstance.getDefault();

  private DoublePublisher imuPlotting;
  private DoubleTopic imuTopic;

  private DoublePublisher gyroPlotting;
  private DoubleTopic gyroTopic;

  private DoubleArrayPublisher imuDataPublisher;
  private DoubleArrayTopic imuData;

  private DoublePublisher voltagePublisher = netTables.getTable("PDH").getDoubleTopic("Voltage").publish();

  private PowerDistribution pdh = new PowerDistribution();

  private double elevatorIKTargetX = Constants.Wrist.startingPositionRotation;
  private double elevatorIKTargetY = Constants.Shoulder.shoulderStartingRotation;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDashboard();

    Constants.log(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);

    Constants.Sensors.calibrate(new Pose3d());

    elevatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              elevatorIKTargetX -= m_secondaryController.getRightX() / 80;
              elevatorIKTargetY -= m_secondaryController.getRightY() / 80;
              Constants.log("Targets: " + elevatorIKTargetX + " " + elevatorIKTargetY);
              //Constants.log(elevatorIKTargetY);
              //elevatorSubsystem.setAdditive(-m_secondaryController.getLeftY() / 60);
              elevatorSubsystem.setShoulder(elevatorIKTargetY);
              elevatorSubsystem.setWrist(elevatorIKTargetX);
              //elevatorSubsystem.goToPosition(elevatorIKTargetY, elevatorIKTargetX, 0);
            },
            elevatorSubsystem));
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
        drivetrainSubsystem.set(-m_driverController.getLeftY(), -m_driverController.getLeftX(), -m_driverController.getRightX());
    }, drivetrainSubsystem));
    //clawSubsystem.setDefaultCommand(new RunCommand(() -> {clawSubsystem.set(-m_secondaryController.getRightY());}, clawSubsystem));
  }

  public void init() {
    Constants.log("Enabling...");
  
    //configureDashboard();
    //drivetrainSubsystem.stop();
  }

  public void periodic() {
    //Constants.Sensors.vision.periodic();
    //imuPlotting.set(Constants.Sensors.getImuRotation3d().getZ());
    imuDataPublisher.set(new double[] {Constants.Sensors.getIMUYawRadians(), -Constants.Sensors.imu.getRate()});
    //svoltagePublisher.set(pdh.getVoltage());
    //imuDataPublisher.set(Constants.Sensors.getImuRotation3d().getZ());
  }

  public void enabledPeriodic() {

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  int pipeline = 0;
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().onTrue(
        new InstantCommand(
            () -> {
              Constants.log("Setting elevator position to max...");
              elevatorSubsystem.set(Constants.Elevator.maxExtension);
            }));
    m_driverController.a().onTrue(
            new InstantCommand(
                () -> {
                  Constants.log("Setting elevator position to min...");
                  elevatorSubsystem.set(Constants.Elevator.minExtension);
                }));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> {Constants.Sensors.resetIMU();}));
    m_driverController.x().onTrue(new InstantCommand(() -> {elevatorSubsystem.logEncoders();}, elevatorSubsystem));
    m_driverController.y().onTrue(new InstantCommand(() -> {
      if (pipeline == 0) {pipeline = 1;}
      else if (pipeline == 1) {pipeline = 0;}
      drivetrainSubsystem.setVisionPipeline(pipeline);}));
  }

  private void configureDashboard() {
    double[] def = {0, 0};

    //imuPlotting = netTables.getDoubleTopic("IMU Gyro Rads").publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    //imuPlotting.setDefault(0);

    //gyroPlotting = netTables.getDoubleTopic("IMU Gyro Rads").publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    //gyroPlotting.setDefault(0);

    imuDataPublisher = netTables.getDoubleArrayTopic("IMU Data | Yaw & Yaw Velocity").publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    imuDataPublisher.setDefault(def);

    SmartDashboard.putData("play", new InstantCommand(() -> { drivetrainSubsystem.play(); }));
    SmartDashboard.putData("pause", new InstantCommand(() -> { drivetrainSubsystem.pause(); }));
    SmartDashboard.putData("stop", new InstantCommand(() -> { drivetrainSubsystem.stop(); }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return ReturnAutonomous.OrientLimelight(drivetrainSubsystem);
  }
}
