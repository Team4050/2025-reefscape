package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
  private StatusSignal<AngularVelocity>[] motorVelocities;
  private StatusSignal<Current>[] motorAppliedCurrent;

  // ******************************************************** Math & Control ******************************************************** //
  private MecanumDriveKinematics kinematics;
  private KalmanFilter<N3, N3, N4> mecanumFieldRelativeKalmanFilter;
  private LinearSystem<N3, N3, N4> mecanumFieldRelativeSystem;
  private LinearQuadraticRegulator<N3, N3, N4> mecanumFieldRelativeLQR;
  private LinearPlantInversionFeedforward<N3, N3, N4> mecanumFF;

  // ******************************************************** Networktables ******************************************************** //
  private boolean useNetworkTables;
  private NetworkTable drivetrainTable;
  private DoubleArrayPublisher appliedCurrent;
  private DoubleArrayPublisher velocity;

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
        loggingLoop = 0;
      } else if (logInfo > 1) {
        loggingLoop = 50;
      }
    }
    FL = new TalonFX(Constants.Drivetrain.FL);
    FR = new TalonFX(Constants.Drivetrain.FR);
    RL = new TalonFX(Constants.Drivetrain.RL);
    RR = new TalonFX(Constants.Drivetrain.RR);

    double maxOutput = 0.3; // Increase in proportion to confidence in driver skill
    leftSideConfig = new TalonFXConfiguration();
    leftSideConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput;
    leftSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
    leftSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    leftSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    leftSideConfig.Audio.AllowMusicDurDisable = true;

    rightSideConfig = new TalonFXConfiguration();
    rightSideConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightSideConfig.MotorOutput.PeakForwardDutyCycle = maxOutput; // Increase in proportion to confidence in driver skill
    rightSideConfig.MotorOutput.PeakReverseDutyCycle = -maxOutput;
    rightSideConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    rightSideConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    rightSideConfig.Audio.AllowMusicDurDisable = true;

    music = new Orchestra();
    music.addInstrument(FL, 0);
    music.addInstrument(FR, 1);
    music.addInstrument(RL, 2);
    music.addInstrument(RR, 3);
    music.loadMusic("E1M1.chrp");

    FL.getConfigurator().apply(leftSideConfig);
    RL.getConfigurator().apply(leftSideConfig);

    FR.getConfigurator().apply(rightSideConfig);
    RR.getConfigurator().apply(rightSideConfig);

    motorVelocities = new StatusSignal[4];
    motorVelocities[0] = FL.getVelocity();
    motorVelocities[1] = FL.getVelocity();
    motorVelocities[2] = FL.getVelocity();
    motorVelocities[3] = FL.getVelocity();

    motorAppliedCurrent = new StatusSignal[4];
    motorAppliedCurrent[0] = FL.getStatorCurrent();
    motorAppliedCurrent[1] = FR.getStatorCurrent();
    motorAppliedCurrent[2] = RL.getStatorCurrent();
    motorAppliedCurrent[3] = RR.getStatorCurrent();

    if (useNetworkTables) {
      drivetrainTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
      appliedCurrent = drivetrainTable.getDoubleArrayTopic("Applied currents | amperes").publish();
      velocity = drivetrainTable.getDoubleArrayTopic("Wheel encoder velocities | rad/s").publish();
    }

    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(0.5588, 0.5588),
            new Translation2d(0.5588, -0.5588),
            new Translation2d(-0.5588, 0.5588),
            new Translation2d(-0.5588, -0.5588));

    double rootHalf = Math.sqrt(2) / 2;
    double d = 0.39513;
    //mecanumChassisRelativeSystem = new LinearSystem<N3, N4, N5>(A, B, C, D);


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
      0.000001, 0, 0,
      0, 0.000001, 0,
      0, 0, 0.000001,
      0, 0, 0.000001,
    };
    double[] dD2 = { //Outputs x inputs
      1, 0, 0,
      0, 1, 0,
      0, 0, 1,
      0, 0, 1,
    };
    Matrix<N3, N3> A2 = new Matrix<N3, N3>(N3.instance, N3.instance, dA2);
    Matrix<N3, N3> B2 = new Matrix<N3, N3>(N3.instance, N3.instance, dB2);
    Matrix<N4, N3> C2 = new Matrix<N4, N3>(N4.instance, N3.instance, dC2);
    Matrix<N4, N3> D2 = new Matrix<N4, N3>(N4.instance, N3.instance, dD2);
    mecanumFieldRelativeSystem = new LinearSystem<N3, N3, N4>(A2, B2, C2, D2); 
    
    Matrix<N3, N1> stateDev = VecBuilder.fill(0.01, 0.01, 0.01);
    Matrix<N4, N1> sensorDev = VecBuilder.fill(0.001, 0.001, 0.001, 0.05);

    mecanumFieldRelativeKalmanFilter = new KalmanFilter<N3, N3, N4>(N3.instance, N4.instance, mecanumFieldRelativeSystem, stateDev, sensorDev, 0.02);

    mecanumFieldRelativeLQR = new LinearQuadraticRegulator<>(mecanumFieldRelativeSystem, VecBuilder.fill(0.05, 0.05, 0.05), VecBuilder.fill(1, 1, 1), d);
    mecanumFF = new LinearPlantInversionFeedforward<>(mecanumFieldRelativeSystem, 0.02);
    //mecanumKalmanFilter.predict();

    //mecanumFF = new LinearPlantInversionFeedforward<N3, N4, N5>(mecanumSystem, 0.02);
    //Matrix FF = mecanumFF.calculate(VecBuilder.fill(2, 2, 0), VecBuilder.fill(2, 2, 0));
    //Matrix xN = mecanumChassisRelativeSystem.calculateX(VecBuilder.fill(1,1, 0), VecBuilder.fill(1, -1, 1, -1), 1);
    //Constants.log("New X:" + xN.toString());
    //Constants.log("FF:" + FF.toString());
  }

  private double moduleMetersPerSecondToKrakenRPM(double mps) {
    double rads = mps * Constants.Drivetrain.moduleGearReduction / Constants.Drivetrain.wheelRadiusMeters;
    return rads * 60 / (2 * Math.PI);
  }

  private double krakenRPMToMetersPerSecond(double rpm) {
    double rads = rpm * 2 * Math.PI / 60;
    return rads * Constants.Drivetrain.wheelRadiusMeters / Constants.Drivetrain.moduleGearReduction;
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

  public double[] getAppliedCurrents() {
    double[] d = {
      motorAppliedCurrent[0].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[1].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[2].refresh().getValue().in(Units.Amps),
      motorAppliedCurrent[3].refresh().getValue().in(Units.Amps)
    };
    return d;
  }

  public Vector<N3> stateSim = VecBuilder.fill(0, 0, 0);
  public Vector<N4> getY() {
    double rh = Math.sqrt(2) / 2;
    double d = 0.39513;
    double[] dB = {
      rh, -rh, -d,
      rh, rh, d,
      rh, rh, -d,
      rh, -rh, d
    }; // y = M * x
    //x = M+ * y
    Matrix<N4, N3> B = new Matrix<N3, N4>(N3.instance, N4.instance, dB).transpose();

    stateSim = new Vector<N3>(mecanumFieldRelativeSystem.calculateX(stateSim, u, 0.02));
    return new Vector<N4>(mecanumFieldRelativeSystem.getC().times(stateSim).plus(mecanumFieldRelativeSystem.getD().times(u)));
    /*
    Vector<N4> motorVelocities = new Vector<N4>(new SimpleMatrix(getEncoders()));
    Vector<N3> inputToChassisVelocity = new Vector<N3>(B.transpose().times(motorVelocities));
    //return VecBuilder.fill(inputToChassisVelocity.get(0), inputToChassisVelocity.get(1), inputToChassisVelocity.get(2), Constants.Sensors.imu.getRate());
    return VecBuilder.fill(stateSim.get(0), stateSim.get(1), stateSim.get(2), stateSim.get(2));*/
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
    FL.set((wheelSpeeds.frontLeftMetersPerSecond));
    FR.set((wheelSpeeds.frontRightMetersPerSecond));
    RL.set((wheelSpeeds.rearLeftMetersPerSecond));
    RR.set((wheelSpeeds.rearRightMetersPerSecond));
  }

  /***
   * Set the speed of the chassis in field-relative m/s
   * @param fieldRelativeSpeeds
   * @return
   */
  public void set(ChassisSpeeds fieldRelativeSpeeds) {
    kinematics.toWheelSpeeds(fieldRelativeSpeeds);
  }

  public void play() {
    music.play();
  }

  public void pause() {
    music.pause();
  }

  public void stop() {
    music.stop();
  }

  public void init() {

  }

  private Vector<N3> u = VecBuilder.fill(0, 0, 0);
  private int loop = 0;
  @Override
  public void periodic() {
    if (useNetworkTables) {
      appliedCurrent.set(getAppliedCurrents());
      velocity.set(getEncoders());  
    }

    //Vector<N3> xN = (Vector<N3>) mecanumChassisRelativeSystem.calculateX(VecBuilder.fill(1,1, 0), VecBuilder.fill(1, -1, 1, -1), 0.02);

    Vector<N4> y = getY();
    Constants.log("Calculated output:" + mecanumFieldRelativeSystem.calculateY(stateSim, u).toString());
    Constants.log("Output simulation:" + y.toString());
    mecanumFieldRelativeKalmanFilter.predict(u, 0.02);
    Constants.log("Predicted:" + mecanumFieldRelativeKalmanFilter.getXhat());
    mecanumFieldRelativeKalmanFilter.correct(u, y);
    Vector<N3> x = new Vector<N3>(mecanumFieldRelativeKalmanFilter.getXhat());
    u = new Vector<N3>(mecanumFieldRelativeLQR.calculate(x, VecBuilder.fill(1, 0, 0)).plus(mecanumFF.calculate(VecBuilder.fill(1, 0, 0))));

    // Constants.log("Slipping...");
    // TODO Auto-generated method stub
    super.periodic();

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
