package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
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
  private Pose3d lastAprilTagSeen = new Pose3d();

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
  private MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(kinematics, Rotation2d.kZero, new MecanumDriveWheelPositions(), Pose2d.kZero, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.45, 0.45, 0.45));

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

  // ******************************************************** Model-based control ******************************************************** //
  // x in meters, y in meters, theta in radians
  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, new Constraints(2, 5));
  private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, new Constraints(2, 5));;
  private ProfiledPIDController thetaController = new ProfiledPIDController(0.3, 0, 0, new Constraints(0.2, 0.01));;

  // ******************************************************** Networktables ******************************************************** //
  private boolean useNetworkTables;
  private NetworkTable drivetrainTable;
  private DoubleArrayPublisher appliedCurrent;
  private DoubleArrayPublisher position;
  private DoubleArrayPublisher velocity;
  private DoubleArrayPublisher referenceVectorPublisher;
  private DoubleArrayPublisher yPublisher;
  private DoubleArrayPublisher xPublisher;
  private DoubleArrayPublisher xHatPublisher;
  private DoubleArrayPublisher uPublisher;

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

    double maxOutput = 1.0; // Increase in proportion to confidence in driver skill
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput;
    leftSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
    leftSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    leftSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    leftSideConfig.Audio.AllowMusicDurDisable = true;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput; // Increase in proportion to confidence in driver skill
    rightSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
    rightSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    rightSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
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

    if (useNetworkTables) {
      drivetrainTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
      appliedCurrent = drivetrainTable.getDoubleArrayTopic("Applied currents | Amps").publish();
      velocity = drivetrainTable.getDoubleArrayTopic("Wheel encoder velocities | radps").publish();
      position = drivetrainTable.getDoubleArrayTopic("Wheel positions | meters").publish();
      xPublisher = drivetrainTable.getDoubleArrayTopic("Simulated state | mps").publish();
      xHatPublisher = drivetrainTable.getDoubleArrayTopic("Filtered state | mps").publish();
      yPublisher = drivetrainTable.getDoubleArrayTopic("Sensor measurements | mps & radps").publish();
      uPublisher = drivetrainTable.getDoubleArrayTopic("Control vector").publish();
      referenceVectorPublisher = drivetrainTable.getDoubleArrayTopic("Target pose").publish();
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
  }

  public Drivetrain(boolean useNetworkTables, int logInfo, Pose2d initalPoseEstimate) {
    this(useNetworkTables, logInfo);
    poseEstimator.resetPose(initalPoseEstimate);
  }

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

  public ChassisSpeeds getFieldRelativeSpeedsMPS() {
    var speeds = kinematics.toChassisSpeeds(getWheelVelocitiesMPS());
    Translation2d d = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    d.rotateBy(Constants.Sensors.getImuRotation2d());
    return new ChassisSpeeds(d.getX(), d.getY(), speeds.omegaRadiansPerSecond);
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

  public Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose3d getLastSeenAprilTag() {
    return lastAprilTagSeen;
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

  public void setVisionPipeline(int index) {
    chassisCam.setPipelineIndex(index);
    Constants.log("Drivetrain - Set limelight pipeline index to " + index);
  }

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
   * Directly set the wheel speeds
   * @param wheelSpeeds
   */
  public void set(MecanumDriveWheelSpeeds wheelSpeeds) {
    //Constants.log("Speed control");
    //Constants.log(wheelSpeeds.frontLeftMetersPerSecond);
    FL.set((wheelSpeeds.frontLeftMetersPerSecond));
    FR.set((wheelSpeeds.frontRightMetersPerSecond));
    RL.set((wheelSpeeds.rearLeftMetersPerSecond));
    RR.set((wheelSpeeds.rearRightMetersPerSecond));
  }
  
  /***
   * Set chassis Voltages
   * @param vx
   * @param vy
   * @param omega
   */
  public void setV(double vx, double vy, double omega) {
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, vy, omega));
    setV(speeds);
  }

  public void setV(MecanumDriveWheelSpeeds wheelSpeeds) {
    FL.setVoltage((wheelSpeeds.frontLeftMetersPerSecond * Constants.Drivetrain.drivetrainMotor.KvRadPerSecPerVolt));
    FR.setVoltage((wheelSpeeds.frontRightMetersPerSecond * Constants.Drivetrain.drivetrainMotor.KvRadPerSecPerVolt));
    RL.setVoltage((wheelSpeeds.rearLeftMetersPerSecond * Constants.Drivetrain.drivetrainMotor.KvRadPerSecPerVolt));
    RR.setVoltage((wheelSpeeds.rearRightMetersPerSecond * Constants.Drivetrain.drivetrainMotor.KvRadPerSecPerVolt));
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

  /***
   * Set the speed of the chassis in field-relative duty cycle
   * @param fieldRelativeSpeeds
   * @return
   */
  public void set(ChassisSpeeds fieldRelativeSpeeds) {
    Rotation2d rot = new Rotation2d(Constants.Sensors.getIMUYawRadians());
    MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, rot));
    set(speeds);
  }

  public void followReference() {
    setVelocity(new ChassisSpeeds(u.get(0), u.get(1), u.get(2)));
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

  }

  private Vector<N3> u = VecBuilder.fill(0, 0, 0);
  private int loop = 0;
  @Override
  public void periodic() {
    //Vector<N3> xN = (Vector<N3>) mecanumChassisRelativeSystem.calculateX(VecBuilder.fill(1,1, 0), VecBuilder.fill(1, -1, 1, -1), 0.02);

    var unreadResults = chassisCam.getAllUnreadResults();
    for (PhotonPipelineResult photonPipelineResult : unreadResults) {
      var estimate = aprilTagPoseEstimator.update(photonPipelineResult);
      if (estimate.isPresent()) {
        Constants.log("Saw tag " + estimate.get().targetsUsed.get(0).getFiducialId());
        lastAprilTagSeen = aprilTags.getTagPose(estimate.get().targetsUsed.get(0).getFiducialId()).get();
        poseEstimator.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
      }
    }

    poseEstimator.update(Constants.Sensors.getImuRotation2d(), getWheelPositionsMeters());

    Vector<N4> y = getY();
    mecanumFieldRelativeKalmanFilter.predict(u, 0.02);
    mecanumFieldRelativeKalmanFilter.correct(u, y);
    Vector<N3> x = new Vector<N3>(mecanumFieldRelativeKalmanFilter.getXhat());
    u = new Vector<N3>(mecanumFieldRelativeLQR.calculate(x, referenceVector));//.plus(mecanumFF.calculate(referenceVector)));

    // Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();

    if (useNetworkTables) {
      appliedCurrent.set(getAppliedCurrents());
      velocity.set(getMotorVelocitiesRadPS());
      MecanumDriveWheelPositions positions = getWheelPositionsMeters();
      Pose2d estimate = poseEstimator.getEstimatedPosition();
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
