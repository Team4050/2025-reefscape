package frc.robot.subsystems;

import java.lang.reflect.Type;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N9;
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

  private PhotonCamera chassisCam = new PhotonCamera("Limelight");
  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(
    AprilTagFields.k2025ReefscapeWelded), 
    PoseStrategy.CLOSEST_TO_LAST_POSE, 
    Constants.Drivetrain.robotToCamera);

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

  private MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(kinematics, Constants.Sensors.getImuRotation3d().toRotation2d(), new MecanumDriveWheelPositions(0, 0, 0, 0), Constants.Sensors.vision.pose.toPose2d());

  // ******************************************************** Model-based control ******************************************************** //
  private boolean useSimulator = false;
  private Vector<N6> referenceVector;
  private Vector<N6> stateEstimate = VecBuilder.fill(0, 0, 0, 0, 0, 0);
  static final Type states = N6.class;
  static final Type inputs = N3.class;
  static final Type outputs = N6.class;
  private KalmanFilter<N6, N3, N6> mecanumFieldRelativeKalmanFilter;
  private UnscentedKalmanFilter<N6, N3, N6> posKalmanFilter;
  private LinearSystem<N6, N3, N6> mecanumFieldRelativeSystem;
  private LinearQuadraticRegulator<N6, N3, N6> mecanumFieldRelativeLQR;
  private LinearPlantInversionFeedforward<N6, N3, N6> mecanumFF;

  // ******************************************************** Model-based control ******************************************************** //
  // x in meters, y in meters, theta in radians
  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, new Constraints(2, 5));
  private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, new Constraints(2, 5));;
  private ProfiledPIDController thetaController = new ProfiledPIDController(0.3, 0, 0, new Constraints(0.2, 0.01));
  private PIDController FLVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController FRVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController RLVelPID = new PIDController(0.3, 0, 0.1);
  private PIDController RRVelPID = new PIDController(0.3, 0, 0.1);

  // ******************************************************** Networktables ******************************************************** //
  private boolean useNetworkTables;
  private NetworkTable drivetrainTable;
  private DoubleArrayPublisher appliedCurrent;
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
      xPublisher = drivetrainTable.getDoubleArrayTopic("Simulated state | mps").publish();
      xHatPublisher = drivetrainTable.getDoubleArrayTopic("Filtered change in state | mps").publish();
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

    // States: xpos, ypos, angle, xvel, yvel, avel
    // Inputs: xaccel, yaccel, angaccel
    // Outputs: x, y, a, vx, vy, va



    double[][] dA2 = { //States x states
      {0, 0, 0, 1, 0, 0,},
      {0, 0, 0, 0, 1, 0,},
      {0, 0, 0, 0, 0, 1,},
      {0, 0, 0, 0, 0, 0,},
      {0, 0, 0, 0, 0, 0,},
      {0, 0, 0, 0, 0, 0,},
    };
    double[][] dB2 = { //States x inputs
    //xa, ya, anga
      {0, 0, 0,}, //xpos
      {0, 0, 0,}, //ypos
      {0, 0, 0,}, //angle
      {1, 0, 0,}, //xvel
      {0, 1, 0,}, //yvel
      {0, 0, 1,}, //avel
    };
    
    double[][] dC2 = { //Outputs x states
      //xpos, ypos, angle, xvel, yvel, avel
      {1, 0, 0, 0, 0, 0,}, //x
      {0, 1, 0, 0, 0, 0,}, //y
      {0, 0, 1, 0, 0, 0,}, //a
      {0, 0, 0, 1, 0, 0,}, //vx
      {0, 0, 0, 0, 1, 0,}, //vy
      {0, 0, 0, 0, 0, 1,}, //va
    };
    double[][] dD2 = { //Outputs x inputs
    //xacc, yacc, angacc
      {0, 0, 0,}, //x
      {0, 0, 0,}, //y
      {0, 0, 0,}, //a
      {1, 0, 0,}, //vx
      {0, 1, 0,}, //vy
      {0, 0, 1,}, //va
    };
    var A2 = new SimpleMatrix(dA2);
    var B2 = new SimpleMatrix(dB2);
    var C2 = new SimpleMatrix(dC2);
    var D2 = new SimpleMatrix(dD2);
    mecanumFieldRelativeSystem = new LinearSystem<N6, N3, N6>(new Matrix<>(A2), new Matrix<>(B2), new Matrix<>(C2), new Matrix<>(D2));
    
    Matrix<N6, N1> stateDev = VecBuilder.fill(0.35, 0.288, 2.098, 0.1, 0.1, 0.1);
    Matrix<N6, N1> sensorDev = VecBuilder.fill(0.1, 0.1, 9, 10, 10, 10);

    mecanumFieldRelativeKalmanFilter = new KalmanFilter<N6, N3, N6>(N6.instance, N6.instance, mecanumFieldRelativeSystem, stateDev, sensorDev, 0.02);

    mecanumFieldRelativeLQR = new LinearQuadraticRegulator<N6, N3, N6>(mecanumFieldRelativeSystem, VecBuilder.fill(0.05, 0.05, 0.05, 0.05, 0.05, 0.05), VecBuilder.fill(1, 1, 1), d);

    Constants.log("Controller gain: " + mecanumFieldRelativeLQR.getK());
    mecanumFF = new LinearPlantInversionFeedforward<>(mecanumFieldRelativeSystem, 0.02);
    referenceVector = VecBuilder.fill(0.5, 0, 0, 0, 0, 0);
  }

  public void setRobotPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
    stateEstimate = VecBuilder.fill(pose.getX(), pose.getY(), pose.getRotation().getRadians(), 0, 0 ,0);
  }

  private double krakenRadToM(double rads) {
    return (rads / Constants.Drivetrain.moduleGearReduction) * Constants.Drivetrain.wheelRadiusMeters;
  }

  public MecanumDriveWheelPositions getModulePositionsMeters() {
    return new MecanumDriveWheelPositions(
    krakenRadToM(motorPositions[0].refresh().getValue().in(Units.Radian)),
    krakenRadToM(motorPositions[1].refresh().getValue().in(Units.Radian)),
    krakenRadToM(motorPositions[2].refresh().getValue().in(Units.Radian)),
    krakenRadToM(motorPositions[3].refresh().getValue().in(Units.Radian)));
  }

  public double[] getEncoders() {
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

  public double[] getWheelVelocitiesMPS() {
    double[] d = getEncoders();
    for (int i = 0; i < d.length; i++) {
      d[i] = krakenRadToM(d[i]);
    }
    return d;
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

  public Drivetrain(boolean useNetworkTables, int logInfo, boolean useSimulator) {
    this(useNetworkTables, logInfo);
    this.useSimulator = true;
  }

  public Vector<N6> getY() {
    //Vector<N3> simulatedMotorVelocities = new Vector<N3>(B.times(stateSim));
    //Constants.log("Sim motor velocities:" + simulatedMotorVelocities);

    //stateSim = new Vector<N3>(mecanumFieldRelativeSystem.calculateX(stateSim, u, 0.02));
    //Vector<N4> variance = new Vector<N4>(new Matrix<N4, N1>(N4.instance, N1.instance, new double[] {Math.random() * 0.001, Math.random() * 0.01, Math.random() * 0.001, Math.random() * 0.05}));
    //return new Vector<N4>(mecanumFieldRelativeSystem.getC().times(stateSim).plus(mecanumFieldRelativeSystem.getD().times(u))).plus(variance);
    //return VecBuilder.fill(inputToChassisVelocity.get(0), inputToChassisVelocity.get(1), inputToChassisVelocity.get(2), Constants.Sensors.imu.getRate());

    if (useSimulator) {
      return stateEstimate.plus(VecBuilder.fill(Math.random() * 0.001, Math.random() * 0.001, Math.random() * 0.001, Math.random() * 0.001, Math.random() * 0.001, Math.random() * 0.001)); // , Math.random() * 0.001, Math.random() * 0.001, Math.random() * 0.001
    }

    double[] d = getWheelVelocitiesMPS();
    ChassisSpeeds s = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(d[0], d[1], d[2], d[3]));
    Pose2d pose = poseEstimator.getEstimatedPosition();
    return VecBuilder.fill(pose.getX(), pose.getY(), pose.getRotation().getRadians(), s.vxMetersPerSecond, s.vyMetersPerSecond, s.omegaRadiansPerSecond);
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
    double[] d = getWheelVelocitiesMPS();
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

  public void setReference(double x, double y, double a, double vx, double vy, double va) {
    referenceVector = VecBuilder.fill(x, y, a, vx, vy, va);
  }

  public void setReference(Pose2d pose, Pose2d velocity) {
    setReference(pose.getX(), pose.getY(), 0, velocity.getX(), velocity.getY(), 0);
  }

  public Pose2d getPoseEstimate() {
    return new Pose2d(stateEstimate.get(0), stateEstimate.get(1), new Rotation2d(stateEstimate.get(2)));
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

    var results = chassisCam.getAllUnreadResults();
    for (PhotonPipelineResult photonPipelineResult : results) {
      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(photonPipelineResult);
      if (pose.isPresent()) {
        poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
      }
    }

    poseEstimator.update(Constants.Sensors.getImuRotation2d(), getModulePositionsMeters());


    var y = getY();
    mecanumFieldRelativeKalmanFilter.predict(u, 0.02);
    mecanumFieldRelativeKalmanFilter.correct(u, y);
    var xhat = new Vector<N6>(mecanumFieldRelativeKalmanFilter.getXhat());
    u = new Vector<N3>(mecanumFieldRelativeLQR.calculate(xhat, referenceVector));//.plus(mecanumFF.calculate(referenceVector)));
    stateEstimate = stateEstimate.plus(xhat);

    set(new ChassisSpeeds(u.get(0), u.get(1), u.get(2)));

    // Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();

    if (useNetworkTables) {
      appliedCurrent.set(getAppliedCurrents());
      velocity.set(getEncoders());
      xPublisher.set(stateEstimate.getData());
      xHatPublisher.set(xhat.getData());
      yPublisher.set(y.getData());
      uPublisher.set(u.getData());
      referenceVectorPublisher.set(referenceVector.getData());
    }

    if (loggingLoop == -1) return;
    loop++;
    if (loop > loggingLoop) {
      Constants.log("State estimate:" + stateEstimate.toString());
      Constants.log("Control vec:" + u.toString());
      loop = 0;
    }
  }
}
