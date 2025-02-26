// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.hazard.HazardJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrainSubsystem = new Drivetrain();
  // private final Elevator elevatorSubsystem = new Elevator();
  // private final Claw clawSubsystem = new Claw();

  private final HazardJoystick m_driverController =
      new HazardJoystick(OperatorConstants.kDriverControllerPort);

  private NetworkTableInstance netTables = NetworkTableInstance.getDefault();

  private final DoublePublisher imuPlotting;
  private DoubleTopic imuTopic;

  private final DoublePublisher gyroPlotting;
  private DoubleTopic gyroTopic;

  private final DoubleArrayPublisher imuDataPublisher;
  private DoubleArrayTopic imuData;

  private NetworkTableInstance netTables = NetworkTableInstance.getDefault();

  private final DoublePublisher imuPlotting;
  private DoubleTopic imuTopic;

  private final DoublePublisher gyroPlotting;
  private DoubleTopic gyroTopic;

  private final DoubleArrayPublisher imuDataPublisher;
  private DoubleArrayTopic imuData;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDashboard();

    imuPlotting = imuTopic.publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    imuPlotting.setDefault(0);

    gyroPlotting = gyroTopic.publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    gyroPlotting.setDefault(0);

    imuDataPublisher = imuData.publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    double[] def = {0, 0};
    imuDataPublisher.setDefault(def);

    Constants.Sensors.calibrate();

    /*m_driverController
        .a()
        .onTrue(
            new RunCommand(
                () -> {
                  elevatorSubsystem.set(1000);
                }));

    elevatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              elevatorSubsystem.setAdditive(m_driverController.getRightY());
            },
            elevatorSubsystem));*/
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      drivetrainSubsystem.set(-m_driverController.getY(), m_driverController.getX(), -m_driverController.getZ());
    }));
  }

  public void init() {
    Constants.log("Enabling...");
    SmartDashboard.putString("test", "drivetrainSubsystem");
    // drivetrainSubsystem.stop();
  }

  public void periodic() {
    imuPlotting.set(Constants.Sensors.getImuRotation3d().getZ());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandJoystick Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  private void configureDashboard() {
    imuTopic = new DoubleTopic(netTables.getTopic("IMU Yaw Accel Rads"));
    gyroTopic = new DoubleTopic(netTables.getTopic("IMU Gyro Rads"));
    SmartDashboard.putData(new RunCommand(() -> { drivetrainSubsystem.play(); }));
    SmartDashboard.putData(new RunCommand(() -> { drivetrainSubsystem.pause(); }));
    SmartDashboard.putData(new RunCommand(() -> { drivetrainSubsystem.stop(); }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
