// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefPIDVoltage;
import frc.robot.commands.AutoLoad;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ChooseAutonomous;
import frc.robot.commands.MoveScoringMechanismTo;
import frc.robot.commands.RumbleController;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private boolean drivetrainFieldOrientedMode = false;
  private boolean elevatorTuningMode = false;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrainSubsystem = new Drivetrain(true, 0);
  private final Elevator elevatorSubsystem = new Elevator(elevatorTuningMode);
  private final Claw clawSubsystem = new Claw();
  private final Climber climberSubsystem = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox m_driverController =
      new HazardXbox(OperatorConstants.kDriverControllerPort);
  private final HazardXbox m_secondaryController =
      new HazardXbox(OperatorConstants.kSecondaryControllerPort);

  private final ChooseAutonomous autoChooser =
      new ChooseAutonomous(drivetrainSubsystem, elevatorSubsystem, clawSubsystem);

  private NetworkTableInstance netTables = NetworkTableInstance.getDefault();
  private DoubleArrayPublisher imuDataPublisher =
      netTables.getTable("IMU").getDoubleArrayTopic("Yaw, Yaw velocity").publish();

  private DoublePublisher voltagePublisher =
      netTables.getTable("PDH").getDoubleTopic("Voltage").publish();

  private PowerDistribution pdh = new PowerDistribution();

  private UsbCamera clawCamera;

  private double elevatorIKTargetX = Constants.Wrist.startingRotationRadians;
  private double elevatorIKTargetY = Constants.Shoulder.startingRotationRadians;

  private double drivetrainTargetX = 0;
  private double drivetrainTargetY = 0;
  private double drivetrainTargetAngle = 0;

  private String dashboardTargetX = "Drivetrain target X";
  private String dashboardTargetY = "Drivetrain target Y";
  private String dashboardTargetAngle = "Drivetrain target angle";

  private String dashboardCurrentX = "Drivetrain estimated X";
  private String dashboardCurrentY = "Drivetrain estimated Y";
  private String dashboardCurrentAngle = "Drivetrain estimated Angle";

  private String IKtargetX = "IK target extension";
  private String IKtargetY = "IK target height";
  private String wristTarget = "Wrist target degrees";

  private double elevatorManualControlScalar =
      1; // Will move elevator setpoint .5 gearbox rotations every second

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Constants.setupDataLog();

    clawCamera = CameraServer.startAutomaticCapture();
    Constants.log("Claw camera description:" + clawCamera.getDescription());
    // clawCamera.setVideoMode(new VideoMode(PixelFormat.kMJPEG, 320, 320, 30));

    pdh.setSwitchableChannel(true); // For radio PoE

    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    for (var i : layout.getTags()) {
      Constants.log(i.ID + " " + i.pose);
    }
    // Configure the trigger bindings
    configureBindings();
    configureDashboard();

    // elevatorSubsystem.resetEncoders();

    Constants.log(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);

    Constants.Sensors.calibrate(new Pose3d());
    Constants.log("IMU angle 1: " + Constants.Sensors.imu.getAngle());

    double[] elevatorStartingXY = elevatorSubsystem.elevatorForwardKinematics();

    // Constants.log(SmartDashboard.putNumberArray(IKtargetX, elevatorStartingXY));
    SmartDashboard.putNumber(IKtargetX, elevatorStartingXY[0]);
    SmartDashboard.putNumber(IKtargetY, elevatorStartingXY[1]);
    SmartDashboard.putNumber(wristTarget, 0);
    SmartDashboard.putData(
        "Calculate offset",
        new InstantCommand(
            () -> {
              elevatorSubsystem.recalibrateArmAbsoluteEncoder();
            }));

    elevatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              elevatorIKTargetX +=
                  m_secondaryController.getRightX() * 10 * 0.02; // Robot viewed from left side
              elevatorIKTargetY -= m_secondaryController.getRightY() * 10 * 0.02;
              SmartDashboard.putNumber(IKtargetX, elevatorIKTargetX);
              SmartDashboard.putNumber(IKtargetY, elevatorIKTargetY);
              // double elevatorExt = m_secondaryController.povUp().getAsBoolean() ? 0.02 : 0;
              // elevatorExt = m_secondaryController.povDown().getAsBoolean() ? -0.02 : elevatorExt;
              // elevatorSubsystem.setElevatorAdditive(elevatorExt);
              elevatorSubsystem.setWristAdditive(m_secondaryController.getLeftX());
              elevatorSubsystem.setShoulderAdditive(-m_secondaryController.getLeftY());
            },
            elevatorSubsystem));

    climberSubsystem.setDefaultCommand(new RunCommand(() -> {}, climberSubsystem));

    drivetrainTargetX = drivetrainSubsystem.getPoseEstimate().getX();
    drivetrainTargetY = drivetrainSubsystem.getPoseEstimate().getY();
    drivetrainTargetAngle = drivetrainSubsystem.getPoseEstimate().getRotation().getDegrees();

    Constants.log("IMU angle 2: " + Constants.Sensors.imu.getAngle());

    drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              if (!drivetrainFieldOrientedMode) {
                double fwdControl = driveControlProcess(-m_driverController.getLeftY());
                double strafeControl = driveControlProcess(-m_driverController.getLeftX());
                double turnControl = driveControlProcess(-m_driverController.getRightX());

                drivetrainSubsystem.set(fwdControl, strafeControl, turnControl);
              } else {
                drivetrainTargetX -= m_driverController.getLeftX() * 0.02;
                drivetrainTargetY -= m_driverController.getLeftY() * 0.02;
                drivetrainTargetAngle -= m_driverController.getRightX() * 0.02 * 180;
                if (drivetrainTargetAngle > 180) {
                  drivetrainTargetAngle -= 360;
                } else if (drivetrainTargetAngle <= -180) {
                  drivetrainTargetAngle += 360;
                }
                Constants.log("IMU angle: " + Constants.Sensors.imu.getAngle());
                SmartDashboard.putNumber(dashboardTargetX, drivetrainTargetX);
                SmartDashboard.putNumber(dashboardTargetY, drivetrainTargetY);
                SmartDashboard.putNumber(dashboardTargetAngle, drivetrainTargetAngle);
              }
            },
            drivetrainSubsystem));

    Constants.log("IMU angle 3: " + Constants.Sensors.imu.getAngle());

    clawSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              if (m_secondaryController.povLeft().getAsBoolean()) {
                if (clawSubsystem.algaeMode) {
                  clawSubsystem.set(-0.3);
                } else {
                  clawSubsystem.set(0.1);
                }
              } else if (m_secondaryController.povRight().getAsBoolean()) {
                if (clawSubsystem.algaeMode) {
                  clawSubsystem.set(0.3);
                } else {
                  clawSubsystem.set(-0.1);
                }
              } else {
                clawSubsystem.set(0);
              }
            },
            clawSubsystem));

    Constants.Sensors.imu.setGyroAngleZ(180);
    Constants.log("IMU angle 4: " + Constants.Sensors.imu.getAngle());
  }

  public double driveControlProcess(double value) {
    return Math.signum(value) * Math.pow(Math.abs(value), 1.3);
  }

  /***
   * Called whenever teleop or auto is enabled
   */
  public void init() {
    Constants.log("Enabling...");

    drivetrainSubsystem.stop();
    elevatorSubsystem.init();
  }

  public void periodic() {
    // Constants.Sensors.vision.periodic();
    // imuPlotting.set(Constants.Sensors.getImuRotation3d().getZ());
    Pose2d estimate = drivetrainSubsystem.getPoseEstimate();
    SmartDashboard.putNumber(dashboardCurrentX, estimate.getX());
    SmartDashboard.putNumber(dashboardCurrentY, estimate.getY());
    SmartDashboard.putNumber(dashboardCurrentAngle, estimate.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "IMU measured heading", Math.toDegrees(Constants.Sensors.getIMUYawRadians()));

    imuDataPublisher.set(
        new double[] {
          Constants.Sensors.getIMUYawRadians(), Constants.Sensors.getIMUYawVelocityRads()
        });
    voltagePublisher.set(pdh.getVoltage());
    // imuDataPublisher.set(Constants.Sensors.getImuRotation3d().getZ());
  }

  public void enabledPeriodic() {}

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

    /********************************************************** Primary **************************************************************************/

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController
        .leftTrigger()
        .toggleOnTrue(
            new AlignToReefPIDVoltage(drivetrainSubsystem, false, clawSubsystem.algaeMode));
    m_driverController
        .rightTrigger()
        .toggleOnTrue(
            new AlignToReefPIDVoltage(drivetrainSubsystem, true, clawSubsystem.algaeMode));
    drivetrainSubsystem.autoTrigger.onFalse(new RumbleController(m_driverController, 0.5));
    m_driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  drivetrainFieldOrientedMode = !drivetrainFieldOrientedMode;
                  Constants.log("Drivetrain FOC Mode " + drivetrainFieldOrientedMode);
                }));
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (!drivetrainFieldOrientedMode) return;

                  var target =
                      new Pose2d(
                          drivetrainTargetX,
                          drivetrainTargetY,
                          new Rotation2d(Math.toRadians(drivetrainTargetAngle)));
                  var current = drivetrainSubsystem.getPoseEstimate();

                  Constants.log(
                      "Driving from "
                          + current.getX()
                          + " "
                          + current.getY()
                          + ", heading of "
                          + current.getRotation().getDegrees()
                          + ", to "
                          + drivetrainTargetX
                          + " "
                          + drivetrainTargetY
                          + ", target heading of "
                          + drivetrainTargetAngle);
                  drivetrainSubsystem.pathfindToPose(target).schedule();
                },
                drivetrainSubsystem));
    /*m_driverController.b().onTrue(
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
                }));*/
    m_driverController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevatorSubsystem.recalibrateArmAbsoluteEncoder();
                }));
    // m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
    // Constants.Sensors.resetIMU(); }));
    m_driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevatorSubsystem.logEncoders();
                },
                elevatorSubsystem));
    m_driverController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (pipeline == 0) {
                    pipeline = 1;
                  } else if (pipeline == 1) {
                    pipeline = 0;
                  }
                  drivetrainSubsystem.setVisionPipeline(pipeline);
                }));

    // m_driverController.povDown().onTrue(new InstantCommand(()-> {
    // climberSubsystem.set(Constants.Climber.deployedPositionRotations); }, climberSubsystem));
    // m_driverController.povUp().onTrue(new InstantCommand(() -> {
    // climberSubsystem.set(Constants.Climber.climbedPositionRotations); }, climberSubsystem));

    m_driverController
        .povDown()
        .whileTrue(
            new RunCommand(
                () -> {
                  climberSubsystem.setAdditive(-0.15);
                },
                climberSubsystem));
    m_driverController
        .povUp()
        .whileTrue(
            new RunCommand(
                () -> {
                  climberSubsystem.setAdditive(0.3);
                },
                climberSubsystem));

    /********************************************************** Secondary **************************************************************************/

    m_secondaryController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  clawSubsystem.setAlgaeMode(true);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  clawSubsystem.setAlgaeMode(false);
                }));
    m_secondaryController.rightBumper().onTrue(new AutoLoad(elevatorSubsystem, clawSubsystem));

    if (elevatorTuningMode) {
      m_secondaryController
          .b()
          .onTrue(
              new InstantCommand(
                  () -> {
                    elevatorSubsystem.goToPosition(
                        elevatorIKTargetX,
                        elevatorIKTargetY,
                        SmartDashboard.getNumber(wristTarget, 0),
                        true);
                  },
                  elevatorSubsystem));
      m_secondaryController
          .a()
          .onTrue(
              new InstantCommand(
                  () -> {
                    elevatorSubsystem.reconfigure();
                  },
                  elevatorSubsystem));
      m_secondaryController
          .x()
          .onTrue(
              new InstantCommand(
                  () -> {
                    elevatorSubsystem.recalibrateArmAbsoluteEncoder();
                  }));
    } else {
      m_secondaryController.y().onTrue(MoveScoringMechanismTo.L1(elevatorSubsystem, clawSubsystem));
      m_secondaryController.b().onTrue(MoveScoringMechanismTo.L2(elevatorSubsystem, clawSubsystem));
      m_secondaryController.a().onTrue(MoveScoringMechanismTo.L3(elevatorSubsystem, clawSubsystem));
      m_secondaryController.x().onTrue(MoveScoringMechanismTo.L4(elevatorSubsystem, clawSubsystem));
    }

    BooleanSupplier isAlgaeMode =
        () -> {
          return clawSubsystem.algaeMode;
        };
    m_secondaryController
        .leftTrigger()
        .onTrue(
            new ConditionalCommand(
                MoveScoringMechanismTo.AlgaeTransport(elevatorSubsystem, clawSubsystem),
                MoveScoringMechanismTo.Transport(elevatorSubsystem, clawSubsystem),
                isAlgaeMode));
    m_secondaryController
        .rightTrigger()
        .onTrue(
            new ConditionalCommand(
                Commands.sequence(
                    MoveScoringMechanismTo.AlgaeScoring(elevatorSubsystem, clawSubsystem),
                    new AutoScore(elevatorSubsystem, clawSubsystem)),
                new AutoScore(elevatorSubsystem, clawSubsystem),
                isAlgaeMode));

    m_secondaryController
        .povUp()
        .whileTrue(
            new RunCommand(
                () -> {
                  elevatorSubsystem.setElevatorAdditive(0.02 * elevatorManualControlScalar);
                },
                elevatorSubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  elevatorSubsystem.setElevatorHeightMM(elevatorSubsystem.getElevatorHeightMM());
                },
                elevatorSubsystem));

    m_secondaryController
        .povDown()
        .whileTrue(
            new RunCommand(
                () -> {
                  elevatorSubsystem.setElevatorAdditive(0.02 * -elevatorManualControlScalar);
                },
                elevatorSubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  elevatorSubsystem.setElevatorHeightMM(elevatorSubsystem.getElevatorHeightMM());
                },
                elevatorSubsystem));

    // m_secondaryController.start().onTrue(new SetSubsystemsToClimbingConfig());
    // m_secondaryController.back().onTrue(new SetSubsystemsToClimbingConfig());
  }

  private void configureDashboard() {
    double[] def = {0, 0};

    // imuPlotting = netTables.getDoubleTopic("IMU Gyro Rads").publish(PubSubOption.sendAll(true),
    // PubSubOption.periodic(0.01));
    // imuPlotting.setDefault(0);

    // gyroPlotting = netTables.getDoubleTopic("IMU Gyro Rads").publish(PubSubOption.sendAll(true),
    // PubSubOption.periodic(0.01));
    // gyroPlotting.setDefault(0);

    imuDataPublisher =
        netTables
            .getDoubleArrayTopic("IMU Data | Yaw & Yaw Velocity")
            .publish(PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    imuDataPublisher.setDefault(def);

    SmartDashboard.putData(
        "play",
        new InstantCommand(
            () -> {
              drivetrainSubsystem.play();
            }));
    SmartDashboard.putData(
        "pause",
        new InstantCommand(
            () -> {
              drivetrainSubsystem.pause();
            }));
    SmartDashboard.putData(
        "stop",
        new InstantCommand(
            () -> {
              drivetrainSubsystem.stop();
            }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getAuto();
  }
}
