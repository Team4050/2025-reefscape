package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  // TODO: check if kraken encoders are integrated or remote
  // ******************************************************** Hardware ******************************************************** //
  private Orchestra music;
  private TalonFX FL;
  private TalonFX FR;
  private TalonFX RL;
  private TalonFX RR;
  private TalonFXConfiguration leftSideConfig;
  private TalonFXConfiguration rightSideConfig;

  // ******************************************************** Sensors ******************************************************** //
  private StatusSignal<Angle>[] motorPositions;
  private StatusSignal<AngularVelocity>[] motorVelocities;
  private StatusSignal<Current>[] motorAppliedCurrent;

  private AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private PhotonCamera chassisCam = new PhotonCamera("Limelight");
  private PhotonPoseEstimator aprilTagPoseEstimator = new PhotonPoseEstimator(aprilTags, PoseStrategy.AVERAGE_BEST_TARGETS, Constants.Drivetrain.robotToCamera);
  public boolean invalidPose = false;
  private int lastAprilTagSeen = 0;
  private Timer timeSinceTagSeen = new Timer();

  // ******************************************************** Math & Control ******************************************************** //
  double rh = Math.sqrt(2) / 2;
  double d = 0.39513;
  // y = M * x
  // x = M+ * y

  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    new Translation2d(0.2794, 0.2794).minus(Constants.Drivetrain.COMOffsetFromWheelbaseCenter),
    new Translation2d(0.2794, -0.2794).minus(Constants.Drivetrain.COMOffsetFromWheelbaseCenter),
    new Translation2d(-0.2794, 0.2794).minus(Constants.Drivetrain.COMOffsetFromWheelbaseCenter),
    new Translation2d(-0.2794, -0.2794).minus(Constants.Drivetrain.COMOffsetFromWheelbaseCenter));
  private MecanumDrivePoseEstimator poseEstimator;

  // ******************************************************** Model-based control ******************************************************** //
  private Vector<N3> referenceVector;
  private KalmanFilter<N3, N3, N4> mecanumFieldRelativeKalmanFilter;
  private LinearSystem<N3, N3, N4> mecanumFieldRelativeSystem;
  private PIDController FLVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController FRVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController RLVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController RRVelPID = new PIDController(0.3, 0, 0.1);
  private LinearQuadraticRegulator<N3, N3, N4> mecanumFieldRelativeLQR;
  private LinearPlantInversionFeedforward<N3, N3, N4> mecanumFF;
  private Vector<N3> stateSim = VecBuilder.fill(0, 0, 0);

  // ******************************************************** PID control ******************************************************** //
  // x in meters, y in meters, theta/z in radians
  private double KsX = 0.03;
  private double KsY = 0.03;
  private double KsZ = 0.03;
  private double KvX = 0.01;
  private double KvY = 0.01;
  private double KvZ = 0.01;
  private PIDController xController = new PIDController(0.3, 0.05, 0.03);
  private PIDController yController = new PIDController(0.3, 0.05, 0.03);
  private ProfiledPIDController thetaController = new ProfiledPIDController(0.3, 0.05, 0.03, new Constraints(0.25, 0.5));
  private HolonomicDriveController holonomicDriveController;
  private PPHolonomicDriveController pathPlannerHolonomicDriveController;
  private TorqueCurrentFOC[] controlRequests = new TorqueCurrentFOC[4];

  // ******************************************************** Networktables ******************************************************** //
  private boolean useNetworkTables;
  private NetworkTable drivetrainTable;
  private DoublePublisher timerPublisher;
  private DoubleArrayPublisher appliedCurrent;
  private DoubleArrayPublisher position;
  private DoubleArrayPublisher velocity;
  private DoubleArrayPublisher referenceVectorPublisher;
  private DoubleArrayPublisher yPublisher;
  private DoubleArrayPublisher xPublisher;
  private DoubleArrayPublisher xHatPublisher;
  private DoubleArrayPublisher uPublisher;

  private String dashboardPosKp = "Drivetrain XY Kp";
  private String dashboardPosKi = "Drivetrain XY Ki";
  private String dashboardPosKd = "Drivetrain XY Kd";
  private String dashboardPosIZone = "Drivetrain XY IZone";

  private String dashboardRotKp = "Drivetrain Angle Kp";
  private String dashboardRotKi = "Drivetrain Angle Ki";
  private String dashboardRotKd = "Drivetrain Angle Kd";
  private String dashboardRotIZone = "Drivetrain XY IZone";

  private String dashobardPathMaxV = "Drive path max velocity (m/s)";
  private String dashboardPathMaxA = "Drive path max accel (m/s^2)";

  private String dashboardPosTolerance = "Drivetrain position tolerance (inches)";
  private String dashboardRotTolerance = "Drivetrain rotation tolerance (degrees)";

  // ******************************************************** Networktables ******************************************************** //
  private int loggingLoop = -1;

  @SuppressWarnings("unchecked")
  /***
   * Constructor
   * @param useNetworkTables
   * @param logInfo 0 = No logging, 1 = logging every loop, 2 or >2 = logging every 50 loops (roughly every second)
   */
  public Drivetrain(boolean useNetworkTables, int logInfo) {
    this.useNetworkTables = useNetworkTables;
    if (logInfo != 0) {
      if (logInfo == 1) {
        loggingLoop = 50;
      } else if (logInfo > 1) {
        loggingLoop = 0;
      }
    }
    FL = new TalonFX(Constants.Drivetrain.FL);
    FR = new TalonFX(Constants.Drivetrain.FR);
    RL = new TalonFX(Constants.Drivetrain.RL);
    RR = new TalonFX(Constants.Drivetrain.RR);

    double maxOutput = 0.5; // Increase in proportion to confidence in driver skill
    NeutralModeValue neutralMode = NeutralModeValue.Brake;
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.NeutralMode = neutralMode;
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput;
    leftSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;

    leftSideConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftSideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftSideConfig.CurrentLimits.SupplyCurrentLimit = 60; // TODO: If drive performance is significantly impacted, return to 70
    leftSideConfig.CurrentLimits.StatorCurrentLimit = 160; // TODO: Decrease if motor heat is excessive after drive practice

    leftSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
    leftSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;

    leftSideConfig.Voltage.PeakForwardVoltage = 12;
    leftSideConfig.Voltage.PeakReverseVoltage = -12;

    leftSideConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leftSideConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    leftSideConfig.Audio.AllowMusicDurDisable = true;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.NeutralMode = neutralMode;
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput;
    rightSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;

    rightSideConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightSideConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightSideConfig.CurrentLimits.SupplyCurrentLimit = 60; // TODO: If drive performance is significantly impacted, return to 70
    rightSideConfig.CurrentLimits.StatorCurrentLimit = 160; // TODO: Decrease if motor heat is excessive after drive practice

    rightSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
    rightSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;

    rightSideConfig.Voltage.PeakForwardVoltage = 12;
    rightSideConfig.Voltage.PeakReverseVoltage = -12;

    rightSideConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rightSideConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

    rightSideConfig.Audio.AllowMusicDurDisable = true;

    music = new Orchestra();
    Constants.log("music status: " + music.loadMusic("E1M1.chrp"));
    music.addInstrument(FL, 0);
    music.addInstrument(FR, 1);
    music.addInstrument(RL, 2);
    music.addInstrument(RR, 3);

    FL.getConfigurator().apply(leftSideConfig);
    RL.getConfigurator().apply(leftSideConfig);

    FR.getConfigurator().apply(rightSideConfig);
    RR.getConfigurator().apply(rightSideConfig);

    motorPositions = new StatusSignal[4];
    motorPositions[0] = FL.getPosition();
    motorPositions[1] = FR.getPosition();
    motorPositions[2] = RL.getPosition();
    motorPositions[3] = RR.getPosition();

    motorVelocities = new StatusSignal[4];
    motorVelocities[0] = FL.getVelocity();
    motorVelocities[1] = FR.getVelocity();
    motorVelocities[2] = RL.getVelocity();
    motorVelocities[3] = RR.getVelocity();

    motorAppliedCurrent = new StatusSignal[4];
    motorAppliedCurrent[0] = FL.getStatorCurrent();
    motorAppliedCurrent[1] = FR.getStatorCurrent();
    motorAppliedCurrent[2] = RL.getStatorCurrent();
    motorAppliedCurrent[3] = RR.getStatorCurrent();

    controlRequests[0] = new TorqueCurrentFOC(0);
    controlRequests[1] = new TorqueCurrentFOC(0);
    controlRequests[2] = new TorqueCurrentFOC(0);
    controlRequests[3] = new TorqueCurrentFOC(0);

    if (useNetworkTables) {
      drivetrainTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
      timerPublisher = drivetrainTable.getDoubleTopic("Time since tag detection").publish();
      appliedCurrent = drivetrainTable.getDoubleArrayTopic("Applied currents | Amps").publish();
      velocity = drivetrainTable.getDoubleArrayTopic("Chassis velocities | mps").publish();
      position = drivetrainTable.getDoubleArrayTopic("Pose estimate").publish();
      xPublisher = drivetrainTable.getDoubleArrayTopic("Simulated state | mps").publish();
      xHatPublisher = drivetrainTable.getDoubleArrayTopic("Filtered state | mps").publish();
      yPublisher = drivetrainTable.getDoubleArrayTopic("Sensor measurements | mps & radps").publish();
      uPublisher = drivetrainTable.getDoubleArrayTopic("Control vector").publish();
      referenceVectorPublisher = drivetrainTable.getDoubleArrayTopic("Reference vector").publish();
    }

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(0.2);


    // xhat = Ax + Bu
    // yhat = Cx + Du
    // u =


    double[] dA2 = { //States x states
      0, 0, 0,
      0, 0, 0,
      0, 0, 0
    };
    double[] dB2 = { //States x inputs
      1, 0, 0,
      0, 1, 0,
      0, 0, 1
    };

    double[] dC2 = { //Outputs x states
      1, 0, 0,
      0, 1, 0,
      0, 0, 1,
      0, 0, 1,
    };
    double[] dD2 = { //Outputs x inputs
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
    };
    Matrix<N3, N3> A2 = new Matrix<N3, N3>(N3.instance, N3.instance, dA2);
    Matrix<N3, N3> B2 = new Matrix<N3, N3>(N3.instance, N3.instance, dB2);
    Matrix<N4, N3> C2 = new Matrix<N4, N3>(N4.instance, N3.instance, dC2);
    Matrix<N4, N3> D2 = new Matrix<N4, N3>(N4.instance, N3.instance, dD2);
    mecanumFieldRelativeSystem = new LinearSystem<N3, N3, N4>(A2, B2, C2, D2);

    Matrix<N3, N1> stateDev = VecBuilder.fill(0.01, 0.01, 0.01);
    Matrix<N4, N1> sensorDev = VecBuilder.fill(0.01, 0.01, 0.1, 0.05);

    mecanumFieldRelativeKalmanFilter = new KalmanFilter<N3, N3, N4>(N3.instance, N4.instance, mecanumFieldRelativeSystem, stateDev, sensorDev, 0.02);

    mecanumFieldRelativeLQR = new LinearQuadraticRegulator<>(mecanumFieldRelativeSystem, VecBuilder.fill(0.05, 0.05, 0.05), VecBuilder.fill(1, 1, 1), d);
    Constants.log("Controller gain: " + mecanumFieldRelativeLQR.getK());
    mecanumFF = new LinearPlantInversionFeedforward<>(mecanumFieldRelativeSystem, 0.02);
    referenceVector = VecBuilder.fill(0.5, 0, 0);

    var rot = Constants.Sensors.getImuRotation2d();
    Constants.log("Drivetrain - Initial IMU angle: " + rot);
    poseEstimator = new MecanumDrivePoseEstimator(kinematics, rot, new MecanumDriveWheelPositions(), Constants.startingPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.45, 0.45, 0.45));

    SmartDashboard.putNumber(dashboardPosKp, 0.6);
    SmartDashboard.putNumber(dashboardPosKi, 0);
    SmartDashboard.putNumber(dashboardPosKd, 0);
    SmartDashboard.putNumber(dashboardPosIZone, 0.3048);
    SmartDashboard.putNumber(dashobardPathMaxV, 12);
    SmartDashboard.putNumber(dashboardPathMaxA, 6);

    SmartDashboard.putNumber(dashboardRotKp, 0.3);
    SmartDashboard.putNumber(dashboardRotKi, 0);
    SmartDashboard.putNumber(dashboardRotKd, 0);
    SmartDashboard.putNumber(dashboardRotIZone, Math.toRadians(20));
    SmartDashboard.putNumber(dashboardPathMaxA, 90);

    SmartDashboard.putNumber(dashboardPosTolerance, 2);
    SmartDashboard.putNumber(dashboardRotTolerance, 5);

    holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);

    pathPlannerHolonomicDriveController = new PPHolonomicDriveController(new PIDConstants(0.4, 0, 0, 0.3048), new PIDConstants(0.3, 0, 0, Math.toRadians(20)));

    AutoBuilder.configure(
      this::getPoseEstimate,
      this::resetPoseEstimate,
      this::getRobotRelativeSpeedsMPS,
      this::drivePathFieldRelative,
      pathPlannerHolonomicDriveController,
      Constants.Drivetrain.mainConfig,
      Constants::alliance,
      this);

    timeSinceTagSeen.start();
  }

  public Drivetrain(boolean useNetworkTables, int logInfo, Pose2d initalPoseEstimate) {
    this(useNetworkTables, logInfo);
    Constants.log("IMU ANGLE: " + Constants.Sensors.getImuRotation2d());
    poseEstimator.resetPosition(Constants.Sensors.getImuRotation2d(), getWheelPositionsMeters(), initalPoseEstimate);
    Constants.log("IMU ANGLE: " + Constants.Sensors.getImuRotation2d());
  }

  public void reconfigure() {
    xController = new PIDController(SmartDashboard.getNumber("Drivetrain X Kp", 0.1), SmartDashboard.getNumber("Drivetrain X Ki", 0), SmartDashboard.getNumber("Drivetrain X Kd", 0));
    yController = new PIDController(SmartDashboard.getNumber("Drivetrain Y Kp", 0.1), SmartDashboard.getNumber("Drivetrain Y Ki", 0), SmartDashboard.getNumber("Drivetrain Y Kd", 0));
    thetaController = new ProfiledPIDController(SmartDashboard.getNumber("Drivetrain Angle Kp", 0.3), SmartDashboard.getNumber("Drivetrain Angle Ki", 0), SmartDashboard.getNumber("Drivetrain Angle Kd", 0), new Constraints(SmartDashboard.getNumber("Drivetrain Angle maxV", 1.57079), 3.14));
    holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);
    holonomicDriveController.setTolerance(new Pose2d(
      SmartDashboard.getNumber("Drivetrain X Tolerance", 2) * 0.0254,
      SmartDashboard.getNumber("Drivetrain Y Tolerance", 2) * 0.0254,
      new Rotation2d(Math.toRadians(SmartDashboard.getNumber("Drivetrain Angle Tolerance", 5)))));

    pathPlannerHolonomicDriveController = new PPHolonomicDriveController(new PIDConstants(
      SmartDashboard.getNumber(dashboardPosKp, 0.1),
      SmartDashboard.getNumber(dashboardPosKi, 0),
      SmartDashboard.getNumber(dashboardPosKd, 0),
      SmartDashboard.getNumber(dashboardPosIZone, 0.3048)), new PIDConstants(
      SmartDashboard.getNumber(dashboardRotKp, 0.1),
      SmartDashboard.getNumber(dashboardRotKi, 0),
      SmartDashboard.getNumber(dashboardRotKd, 0),
      SmartDashboard.getNumber(dashboardRotIZone, Math.toRadians(20))
    ));
  }

  // ******************************************************** Getters ******************************************************** //

  private double krakenRadToM(double rads) {
    return (rads / Constants.Drivetrain.moduleGearReduction) * Constants.Drivetrain.wheelRadiusMeters;
  }

  public double[] getMotorPositionRadians() {
    double d[] = {
      motorPositions[0].refresh().getValue().in(Units.Radians),
      motorPositions[1].refresh().getValue().in(Units.Radians),
      motorPositions[2].refresh().getValue().in(Units.Radians),
      motorPositions[3].refresh().getValue().in(Units.Radians),
    };
    return d;
  }

  public double[] getMotorVelocitiesRadPS() {
    // ChassisSpeeds s = kinematics.toChassisSpeeds(new
    // MecanumDriveWheelSpeeds(FL.().getValueAsDouble(), 0, 0, 0))
    double[] d = {
      motorVelocities[0].refresh().getValue().in(Units.RadiansPerSecond),
      motorVelocities[1].refresh().getValue().in(Units.RadiansPerSecond),
      motorVelocities[2].refresh().getValue().in(Units.RadiansPerSecond),
      motorVelocities[3].refresh().getValue().in(Units.RadiansPerSecond)
    };
    return d;
  }

  public MecanumDriveWheelPositions getWheelPositionsMeters() {
    double[] d = getMotorPositionRadians();
    for (int i = 0; i < d.length; i++) {
      d[i] = krakenRadToM(d[i]);
    }
    return new MecanumDriveWheelPositions(d[0], d[1], d[2], d[3]);
  }


  public MecanumDriveWheelSpeeds getWheelVelocitiesMPS() {
    double[] d = getMotorVelocitiesRadPS();
    for (int i = 0; i < d.length; i++) {
      d[i] = krakenRadToM(d[i]);
    }
    return new MecanumDriveWheelSpeeds(d[0], d[1], d[2], d[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeedsMPS() {
    return kinematics.toChassisSpeeds(getWheelVelocitiesMPS());
  }

  public ChassisSpeeds getFieldRelativeSpeedsMPS() {
    var speeds = getRobotRelativeSpeedsMPS();
    Translation2d d = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    d.rotateBy(Constants.Sensors.getImuRotation2d());
    return new ChassisSpeeds(d.getX(), d.getY(), speeds.omegaRadiansPerSecond);
  }

  public ChassisSpeeds toRobotRelative(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPoseEstimate().getRotation());
  }

  public MecanumDriveWheelSpeeds toModuleSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    return kinematics.toWheelSpeeds(robotRelativeSpeeds);
  }

  public double[] getAppliedCurrents() {
    double[] d = {
      motorAppliedCurrent[0].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[1].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[2].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[3].refresh().getValue().in(Units.Amps)
    };
    return d;
  }

  /***
   * Returns the estimated pose
   * @return
   */
  public Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  /***
   * Resets the pose estimator's pose
   * @param pose
   */
  public void resetPoseEstimate(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public Optional<Pose3d> getLastSeenAprilTag() {
    return aprilTags.getTagPose(lastAprilTagSeen);
  }

  public double getTimeSinceLastSeenTag() {
    return timeSinceTagSeen.get();
  }

  public Vector<N4> getY() {
    //Vector<N4> simulatedMotorVelocities = new Vector<N4>(B.times(stateSim));
    //Constants.log("Sim motor velocities:" + simulatedMotorVelocities);

    //stateSim = new Vector<N3>(mecanumFieldRelativeSystem.calculateX(stateSim, u, 0.02));
    //Vector<N4> variance = new Vector<N4>(new Matrix<N4, N1>(N4.instance, N1.instance, new double[] {Math.random() * 0.001, Math.random() * 0.01, Math.random() * 0.001, Math.random() * 0.05}));
    //return new Vector<N4>(mecanumFieldRelativeSystem.getC().times(stateSim).plus(mecanumFieldRelativeSystem.getD().times(u))).plus(variance);
    //return VecBuilder.fill(inputToChassisVelocity.get(0), inputToChassisVelocity.get(1), inputToChassisVelocity.get(2), Constants.Sensors.imu.getRate());
    var d = getWheelVelocitiesMPS();
    ChassisSpeeds s = kinematics.toChassisSpeeds(d);
    return VecBuilder.fill(s.vxMetersPerSecond, s.vyMetersPerSecond, s.omegaRadiansPerSecond, Constants.Sensors.getIMUYawVelocityRads());
  }

    /***
   * Set the speed of the chassis in field-relative duty cycle
   * @param fieldRelativeSpeeds
   * @return
   */
  public void toRobotModuleRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    Rotation2d rot = getPoseEstimate().getRotation();
    var combined = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, rot);
    var ff = new ChassisSpeeds(Math.signum(combined.vxMetersPerSecond) * 0.03, Math.signum(combined.vyMetersPerSecond) * 0.05, Math.signum(combined.omegaRadiansPerSecond) * 0.003);
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(combined.plus(ff));
    set(speeds);
  }

  public MecanumDriveWheelSpeeds calculateFeedforward(ChassisSpeeds robotRelativeSpeeds) {
    var ff = new ChassisSpeeds(Math.signum(robotRelativeSpeeds.vxMetersPerSecond) * 0.03, Math.signum(robotRelativeSpeeds.vyMetersPerSecond) * 0.05, Math.signum(robotRelativeSpeeds.omegaRadiansPerSecond) * 0.003);
    return kinematics.toWheelSpeeds(ff);
  }

  public void setVisionPipeline(int index) {
    chassisCam.setPipelineIndex(index);
    Constants.log("Drivetrain - Set limelight pipeline index to " + index);
  }

  public Command pathfindToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, new PathConstraints(12, 2, Math.PI, Math.PI));
  }

  // ******************************************************** Setters ******************************************************** //

  // ************************************ Set motor direct ************************************ //

  // **************************** Robot relative **************************** //

  /***
   * Set the chassis speed relative to the current robot orientation
   * @param fwdSpeed
   * @param strafeSpeed
   * @param angularSpeed
   */
  public void set(double fwdSpeed, double strafeSpeed, double angularSpeed) {
    MecanumDriveWheelSpeeds speeds =
        kinematics.toWheelSpeeds(new ChassisSpeeds(fwdSpeed, strafeSpeed, angularSpeed));
    set(speeds);
  }

      /***
   * Set the speed of the chassis in robot-relative duty cycle
   * @param fieldRelativeSpeeds
   * @return
   */
  public void set(ChassisSpeeds robotRelativeSpeeds) {
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(robotRelativeSpeeds);
    set(speeds);
  }


  /***
   * Directly set the wheel speeds
   * @param wheelSpeeds
   */
  public void set(MecanumDriveWheelSpeeds wheelSpeeds) {
    FL.set(wheelSpeeds.frontLeftMetersPerSecond);
    FR.set(wheelSpeeds.frontRightMetersPerSecond);
    RL.set(wheelSpeeds.rearLeftMetersPerSecond);
    RR.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  /***
   * Set chassis Voltages
   * @param vx
   * @param vy
   * @param theta
   */
  public void setVoltage(double vx, double vy, double theta) {
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, vy, theta));
    setVoltage(speeds);
  }

  public void setVoltage(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    setVoltage(wheelSpeeds);
  }

  public void setVoltage(MecanumDriveWheelSpeeds wheelSpeeds) {
    FL.setVoltage(wheelSpeeds.frontLeftMetersPerSecond);
    FR.setVoltage(wheelSpeeds.frontRightMetersPerSecond);
    RL.setVoltage(wheelSpeeds.rearLeftMetersPerSecond);
    RR.setVoltage(wheelSpeeds.rearRightMetersPerSecond);
  }

  // **************************** Field relative **************************** //

  /***
   * Set the speed of the chassis in field-relative duty cycle
   * @param fieldRelativeSpeeds
   * @return
   */
  public void setFieldRelative(ChassisSpeeds fieldRelativeSpeeds, ChassisSpeeds robotRelativeFeedforward) {
    Rotation2d rot = getPoseEstimate().getRotation();
    var combined = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, rot);
    var ff = new ChassisSpeeds(Math.signum(combined.vxMetersPerSecond) * 0.03, Math.signum(combined.vyMetersPerSecond) * 0.05, Math.signum(combined.omegaRadiansPerSecond) * 0.003);
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(combined.plus(ff));
    set(speeds);
  }

    /***
   * Set the speed of the chassis in field-relative voltages
   * @param fieldRelativeSpeeds
   * @return
   */
  public void setVoltageFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseEstimate().getRotation());
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassisSpeeds);
    setVoltage(speeds);
  }

  // ************************************ Set setpoints ************************************ //

  // **************************** Robot relative **************************** //

  public void setTorqueCurrents(MecanumDriveWheelSpeeds wheelTorqueCurrents) {
    controlRequests[0].withOutput(wheelTorqueCurrents.frontLeftMetersPerSecond);
    controlRequests[1].withOutput(wheelTorqueCurrents.frontRightMetersPerSecond);
    controlRequests[2].withOutput(wheelTorqueCurrents.rearLeftMetersPerSecond);
    controlRequests[3].withOutput(wheelTorqueCurrents.rearRightMetersPerSecond);

    FL.setControl(controlRequests[0]);
    FR.setControl(controlRequests[1]);
    RL.setControl(controlRequests[2]);
    RR.setControl(controlRequests[2]);
  }

  /***
   * Set closed-loop velocity control (Not tuned - do not use)
   * @param speeds
   */
  @Deprecated
  public void setVelocity(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double[] d = {};//getWheelVelocitiesMPS();
    FL.set(FLVelPID.calculate(d[0], wheelSpeeds.frontLeftMetersPerSecond));
    FR.set(FRVelPID.calculate(d[1], wheelSpeeds.frontRightMetersPerSecond));
    RL.set(RLVelPID.calculate(d[2], wheelSpeeds.rearLeftMetersPerSecond));
    RR.set(RRVelPID.calculate(d[3], wheelSpeeds.rearRightMetersPerSecond));
  }

  // **************************** Field relative **************************** //

  /***
   * Use the Holonomic Drive Controller to follow a rajectory
   * @param targetPose Sampled pose from trajectory
   * @param targetLinearVelocity Sampled velocity from trajectory
   */
  public void driveFieldRelative(Pose2d targetPose, double targetLinearVelocity) {
    Constants.log("Holonomic drive desired state: " + targetPose.toString());
    var diff = targetPose.getRotation().rotateBy(getPoseEstimate().getRotation().unaryMinus());
    var rotationWrappedPose = new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), diff);
    var speeds = holonomicDriveController.calculate(rotationWrappedPose, targetPose, targetLinearVelocity, targetPose.getRotation());
    Constants.log(speeds);
    setFieldRelative(speeds, new ChassisSpeeds());
  }

  public void drivePathFieldRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    var FFAmps = feedforwards.torqueCurrentsAmps();
    var robotRelativeSpeeds = kinematics.toWheelSpeeds(speeds);

    controlRequests[0].withOutput(FFAmps[0] + robotRelativeSpeeds.frontLeftMetersPerSecond);
    controlRequests[1].withOutput(FFAmps[1] + robotRelativeSpeeds.frontRightMetersPerSecond);
    controlRequests[2].withOutput(FFAmps[2] + robotRelativeSpeeds.rearLeftMetersPerSecond);
    controlRequests[3].withOutput(FFAmps[3] + robotRelativeSpeeds.rearRightMetersPerSecond);

    Constants.log("Torque current FL: " + controlRequests[0].getOutputMeasure().in(Amps));

    FL.setControl(controlRequests[0]);
    FR.setControl(controlRequests[1]);
    RL.setControl(controlRequests[2]);
    RR.setControl(controlRequests[3]);
  }

  public void followReference() {
    setVoltageFieldRelative(new ChassisSpeeds(u.get(0), u.get(1), u.get(2)));
  }

  public void setReference(double vx, double vy, double va) {
    referenceVector = VecBuilder.fill(vx, vy, va);
  }

  public void setReference(Pose2d velocity) {
    setReference(velocity.getX(), velocity.getY(), velocity.getRotation().getRadians());
  }

  public void play() {
    Constants.log("Playing...");
    Constants.log(music.play());
  }

  public void pause() {
    Constants.log("Pausing...");
    music.pause();
  }

  public void stop() {
    Constants.log("Stopping...");
    music.stop();
  }

  public void init() {
    stop();
  }

  // ******************************************************** Periodic ******************************************************** //

  private Vector<N3> u = VecBuilder.fill(0, 0, 0);
  private int loop = 0;
  @Override
  public void periodic() {
    //Vector<N3> xN = (Vector<N3>) mecanumChassisRelativeSystem.calculateX(VecBuilder.fill(1,1, 0), VecBuilder.fill(1, -1, 1, -1), 0.02);

    var unreadResults = chassisCam.getAllUnreadResults();
    double closestMeters = 100;
    for (PhotonPipelineResult photonPipelineResult : unreadResults) {
      var estimate = aprilTagPoseEstimator.update(photonPipelineResult);
      if (estimate.isPresent()) {
        //Constants.log("Saw tag " + estimate.get().targetsUsed.get(0).getFiducialId());
        for (int i = 0; i < estimate.get().targetsUsed.size(); i++) {
          double distanceToTag = estimate.get().targetsUsed.get(i).bestCameraToTarget.getTranslation().getNorm();
          if (distanceToTag < closestMeters) {
            closestMeters = distanceToTag;
            timeSinceTagSeen.reset();
            lastAprilTagSeen = estimate.get().targetsUsed.get(i).getFiducialId();
          }
        }

        String message = "Saw tags";
        /*for (PhotonTrackedTarget t : (PhotonTrackedTarget[])estimate.get().targetsUsed.toArray()) {
          message += (" " + t .fiducialId);
        }
        message += timeSinceLastTag.get();
        Constants.logStringToFile(message);*/

        var pose = estimate.get().estimatedPose;
        Constants.limelightDataLogEntry.append(new double[] {
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          pose.getRotation().getQuaternion().getW(),
          pose.getRotation().getQuaternion().getX(),
          pose.getRotation().getQuaternion().getY(),
          pose.getRotation().getQuaternion().getZ()});
        poseEstimator.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
      }
    }

    poseEstimator.update(Constants.Sensors.getImuRotation2d(), getWheelPositionsMeters());
    var pose = getPoseEstimate();
    //Constants.estimatedPositionLogEntry.append(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    if (pose.getX() < 0 || pose.getX() > 17.5 || pose.getY() < 0 || pose.getY() > 8) {
      Constants.log("Invalid pose estimate");
      invalidPose = true;
    } else {
      invalidPose = false;
    }

    Vector<N4> y = getY();
    mecanumFieldRelativeKalmanFilter.predict(u, 0.02);
    mecanumFieldRelativeKalmanFilter.correct(u, y);
    Vector<N3> x = new Vector<N3>(mecanumFieldRelativeKalmanFilter.getXhat());
    u = new Vector<N3>(mecanumFieldRelativeLQR.calculate(x, referenceVector));//.plus(mecanumFF.calculate(referenceVector)));

    // Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();

    if (useNetworkTables) {
      timerPublisher.set(timeSinceTagSeen.get());
      appliedCurrent.set(getAppliedCurrents());
      ChassisSpeeds velocityField = getFieldRelativeSpeedsMPS();
      velocity.set(new double[] {velocityField.vxMetersPerSecond, velocityField.vyMetersPerSecond, velocityField.omegaRadiansPerSecond});
      MecanumDriveWheelPositions positions = getWheelPositionsMeters();
      Pose2d estimate = getPoseEstimate();
      position.set(new double[] {estimate.getX(), estimate.getY(), estimate.getRotation().getRadians()});
      xPublisher.set(stateSim.getData());
      xHatPublisher.set(x.getData());
      yPublisher.set(y.getData());
      uPublisher.set(u.getData());
      referenceVectorPublisher.set(referenceVector.getData());
    }

    if (loggingLoop == -1) return;
    loop++;
    if (loop > loggingLoop) {
      Constants.log("State simulation:" + stateSim.toString());
      Constants.log("State estimate:" + x.toString());
      Constants.log("Control vec:" + u.toString());
      loop = 0;
    }
  }
}
