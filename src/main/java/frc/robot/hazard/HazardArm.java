package frc.robot.hazard;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class HazardArm {
  private HazardSparkMax motor;
  private ArmFeedforward feedforward;
  private TrapezoidProfile velocityCurve;
  private PIDController feedback;
  private PIDController velocityFeedback;
  private ProfiledPIDController profiledFeedback;
  private double offset = 0;
  private double setpoint = 0;
  private String name;
  private DoublePublisher setpointPublisher;
  private DoublePublisher setpointVelocityPublisher;
  private DoublePublisher feedbackPublisher;
  private DoublePublisher profiledFeedbackPublisher;
  private DoublePublisher velocityFeedbackPublisher;
  private DoublePublisher feedforwardPublisher;
  private DoublePublisher uPublisher;
  private double maxEffort = 12;
  private boolean tuneUsingDashboard = false;
  private boolean profiledControl = true;
  private boolean overridePID = false;
  private boolean useAbsoluteEncoder = false;
  private boolean stop = false;

  /***
   * Constructor. All anglular units are in radians, all control units are in volts
   * @param motor
   * @param Ks
   * @param Kg
   * @param Kv
   * @param Kp
   * @param Ki
   * @param Kd
   * @param maxV
   * @param maxA
   * @param name
   * @param tuneUsingDashboard
   */
  public HazardArm(
      HazardSparkMax motor,
      double tolerance,
      boolean useAbsoluteEncoder,
      double Ks,
      double Kg,
      double Kv,
      double Kp,
      double Ki,
      double Kd,
      double maxV,
      double maxA,
      String name,
      boolean tuneUsingDashboard) {
    this.motor = motor;
    this.name = name;
    this.useAbsoluteEncoder = useAbsoluteEncoder;
    this.tuneUsingDashboard = tuneUsingDashboard;
    velocityCurve = new TrapezoidProfile(new Constraints(maxV, maxA));
    feedforward = new ArmFeedforward(Ks, Kg, Kv);
    feedback = new PIDController(Kp, Ki, Kd, 0.02);
    feedback.setIntegratorRange(-2, 2);
    feedback.setTolerance(tolerance);
    feedback.setIZone(Math.toRadians(5));

    velocityFeedback = new PIDController(0.5, 0, 0);
    velocityFeedback.setIZone(10);

    profiledFeedback = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxV, maxA), 0.02);
    profiledFeedback.setIntegratorRange(-2, 2);
    profiledFeedback.setTolerance(tolerance);
    profiledFeedback.setIZone(Math.toRadians(20));
    double motorPosition = motor.getPositionRadians();
    if (useAbsoluteEncoder) {
      motorPosition = motor.getAbsPositionRadians();
      Constants.log("Setting relative encoder equal to absolute encoder");
      motor.setEncoderRadians(motorPosition);
      Constants.log("Relative pos: " + motor.getPositionRadians() + ", absolute pos: " + motor.getAbsPositionRadians());
    }
    profiledFeedback.reset(motorPosition, 0);

    if (tuneUsingDashboard) {
      Boolean fail = false;
      fail |= !SmartDashboard.putNumber(name + " Ks", Ks);
      fail |= !SmartDashboard.putNumber(name + " Kg", Kg);
      fail |= !SmartDashboard.putNumber(name + " Kv", Kv);

      fail |= !SmartDashboard.putNumber(name + " Kp", Kp);
      fail |= !SmartDashboard.putNumber(name + " Ki", Ki);
      fail |= !SmartDashboard.putNumber(name + " Kd", Kd);

      fail |= !SmartDashboard.putNumber(name + " maxV", maxV);
      fail |= !SmartDashboard.putNumber(name + " maxA", maxA);
      if (fail) {
        DriverStation.reportError("Failed to push all tuning values to the dashboard!", true);
      }
      ;
    }

    var table = NetworkTableInstance.getDefault().getTable(name);
    setpointPublisher =
    table.getDoubleTopic("Actual Setpoint").publish();
    setpointVelocityPublisher = table.getDoubleTopic("Setpoint Velocity").publish();
    feedbackPublisher =
      table.getDoubleTopic("PID").publish();
    profiledFeedbackPublisher =
      table.getDoubleTopic("Profiled PID").publish();
    velocityFeedbackPublisher =
      table.getDoubleTopic("Velocity PID").publish();
    feedforwardPublisher =
      table.getDoubleTopic("Feedforward").publish();
    uPublisher =
    table.getDoubleTopic("Control effort").publish();
  }

  public HazardArm(
    HazardSparkMax motor,
    double tolerance,
    boolean useAbsoluteEncoder,
    double Ks,
    double Kg,
    double Kv,
    double Kp,
    double Ki,
    double Kd,
    double maxV,
    double maxA,
    String name,
    boolean tuneUsingDashboard,
    boolean profiled) {
    this(motor, tolerance, useAbsoluteEncoder, Ks, Kg, Kv, Kp, Ki, Kd, maxV, maxA, name, tuneUsingDashboard);
    this.profiledControl = profiled;
  }

  public HazardArm(
    HazardSparkMax motor,
    double tolerance,
    boolean useAbsoluteEncoder,
    double Ks,
    double Kg,
    double Kv,
    double Kp,
    double Ki,
    double Kd,
    double maxV,
    double maxA,
    String name,
    boolean tuneUsingDashboard,
    boolean profiled,
    double maxControlEffort) {
    this(motor, tolerance, useAbsoluteEncoder, Ks, Kg, Kv, Kp, Ki, Kd, maxV, maxA, name, tuneUsingDashboard);
    this.profiledControl = profiled;
    this.maxEffort = maxControlEffort;
  }

  //************************************************ Getters ************************************************//


  public HazardSparkMax getMotor() {
    return motor;
  }

  public boolean underLoad() {
    return motor.isUnderLoad();
  }

  public boolean atReference() {
    return feedback.atSetpoint();
  }

  public double getPositionRadians() {
    if (useAbsoluteEncoder) {
      return motor.getAbsPositionRadians();
    }
    return motor.getPositionRadians();
  }

  public double getVelocityRadPS() {
    return motor.getVelocityRadPS();
  }

  //************************************************ Setters ************************************************//

  //**************************** Manual ****************************//

  public void set(double dutyCycle) {
    overridePID = true;
    motor.set(dutyCycle);
  }

  public void setVoltage(double voltage) {
    overridePID = true;
    motor.setControl(voltage, ControlType.kVoltage);
  }

  public void setBrake(boolean brake) {
    Constants.log(name + " motor set to brake mode: " + brake);
    motor.setBrakeMode(brake);
  }

  //**************************** PID ****************************//

  /***
   * Sets the arm setpoint in radians
   * @param value
   */
  public void setpoint(double value) {
    setpoint = value;
    motor.setSetpointPublishingOnly(setpoint);
  }

  /***
   * Sets a new Kg value e.g. (coral/no coral)
   * @param Kg
   */
  public void setLoad(double Kg) {
    Constants.log(name + " load changed from " + feedforward.getKg() + " to " + Kg);
    feedforward.setKg(Kg);
  }

  public void setOffsetAngleRadians(double offset) {
    this.offset = offset;
  }

  //**************************** Safety ****************************//

  public double checkEncoderDifference() {
    return motor.getPositionRadians() - motor.getAbsPositionRadians();
  }

  public double checkVelocityDifference() {
    return motor.getVelocityRadPS() - motor.getAbsVelocityRadPS();
  }

  public void stop() {
    Constants.log(name + " stopped");
    stop = true;
  }

  public void unstop() {
    Constants.log(name + " reenabled");
    stop = false;
  }

  //************************************************ Periodic ************************************************//

  public void periodic() {
    if (stop) {
      motor.set(0);
      return;
    }
    double motorPosition = motor.getPositionRadians();
    double motorVelocity = motor.getVelocityRadPS();
    if (useAbsoluteEncoder) {
      double diff = checkEncoderDifference();
      if (Math.abs(diff) > Math.toRadians(20)) {
        //Constants.log(name + " - Something caused relative encoder to drift away from absolute encoder");
      }
      double vDiff = checkVelocityDifference();
      if (Math.abs(vDiff) > Math.toRadians(90)) {
        //Constants.log(name + " - Something caused relative encoder velocity to be decoupled from absolute encoder velocity");
      } else if (Math.abs(vDiff) > Math.toRadians(20)) {
        //Constants.log(name + " - Major backlash in mechanism detected");
      }

      motorPosition = motor.getAbsPositionRadians();
      motorVelocity = motor.getAbsVelocityRadPS();
    }
    if (overridePID) {
      setpoint = motorPosition;
      overridePID = false;
    } else {
      double ff = feedforward.calculate(offset + setpoint, 0);
      double fb = feedback.calculate(motorPosition, setpoint);
      double vfb = 0;
      feedforwardPublisher.set(ff);
      feedbackPublisher.set(fb);
      if (profiledControl) {
        ff = feedforward.calculate(setpoint, profiledFeedback.getSetpoint().velocity);
        fb = profiledFeedback.calculate(motorPosition, new State(setpoint, 0));
        setpointPublisher.set(profiledFeedback.getSetpoint().position);
        setpointVelocityPublisher.set(profiledFeedback.getSetpoint().velocity);
        vfb = velocityFeedback.calculate(motorVelocity, profiledFeedback.getSetpoint().velocity);
      }
      profiledFeedbackPublisher.set(fb);
      velocityFeedbackPublisher.set(vfb);
      SmartDashboard.putNumber(name + " rotation", Math.toDegrees(motorPosition));
      SmartDashboard.putNumber(name + " setpoint", Math.toDegrees(setpoint));
      double u = ff + fb + vfb;
      if (u < -maxEffort || u > maxEffort) {
        Constants.log("Control effort exceeding limit");
        u = MathUtil.clamp(u, -maxEffort, maxEffort);
        motor.setControl(u, ControlType.kVoltage);
      } else {
        motor.setControl(u, ControlType.kVoltage);
      }
      uPublisher.set(u);
    }
    motor.publishToNetworkTables();
  }

  /***
   * Reconfigure the feedforward and feedback controllers using values input on the dashboard. DOES NOT SAVE INPUT VALUES! YOU MUST RECORD THEM ON YOUR OWN!
   */
  public void reconfigure() {
    double Ks = SmartDashboard.getNumber(name + " Ks", 0);
    double Kg = SmartDashboard.getNumber(name + " Kg", 0);
    double Kv = SmartDashboard.getNumber(name + " Kv", 0);

    double Kp = SmartDashboard.getNumber(name + " Kp", 0);
    double Ki = SmartDashboard.getNumber(name + " Ki", 0);
    double Kd = SmartDashboard.getNumber(name + " Kd", 0);

    double maxV = SmartDashboard.getNumber(name + " maxV", 0);
    double maxA = SmartDashboard.getNumber(name + " maxA", 0);
    Constants.log("Feedforwards: " + Ks + " " + Kg + " " + Kv);
    Constants.log("Control: " + Kp + " " + Ki + " " + Kd);
    Constants.log("Max V and A" + maxV + " " + maxA);
    feedforward.setKs(Ks);
    feedforward.setKg(Kg);
    feedforward.setKv(Kv);
    feedback.setPID(Kp, Ki, Kd);
    profiledFeedback.setPID(Kp, Ki, Kd);
    profiledFeedback.setConstraints(new Constraints(maxV, maxA));
  }
}
