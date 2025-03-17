package frc.robot.hazard;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
  public boolean manualControl = false;

    private HazardSparkMax motor;
    private ArmFeedforward feedforward;
    private TrapezoidProfile velocityCurve;
    private ProfiledPIDController feedback;
    private double setpoint = 0;
    private boolean limitRotation = true;
    private double softLimitMin = Double.NEGATIVE_INFINITY;
    private double softLimitMax = Double.POSITIVE_INFINITY;
    private double hardLimitMin = Double.NEGATIVE_INFINITY;
    private double hardLimitMax = Double.POSITIVE_INFINITY;
    private double Kg = 0;
    private double dynamicLoadKg = 0;
    private String name;
    private DoublePublisher voltagePIDOutput;
    private DoublePublisher velocityFFtarget;
    private boolean tuneUsingDashboard = false;
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
    public HazardArm(HazardSparkMax motor, double tolerance, boolean useAbsoluteEncoder, double Ks, double Kg, double Kv, double Kp, double Ki, double Kd, double maxV, double maxA, String name) {
        this.motor = motor;
        this.name = name;
        this.useAbsoluteEncoder = useAbsoluteEncoder;
        this.Kg = Kg;
        velocityCurve = new TrapezoidProfile(new Constraints(maxV, maxA));
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        feedback = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxV, maxA));
        feedback.setIntegratorRange(-1.5, 1.5);
        feedback.setIZone(Math.toRadians(10));
        feedback.setTolerance(tolerance);
        double motorPosition = motor.getPositionRadians();
        if (useAbsoluteEncoder) {
          motorPosition = motor.getAbsPositionRadians();
        }
        feedback.reset(motorPosition, 0);

        voltagePIDOutput = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("PID Output").publish();
        velocityFFtarget = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("Velocity feedforward").publish();
    }

    public HazardArm(HazardSparkMax motor, double tolerance, boolean useAbsoluteEncoder, double Ks, double Kg, double Kv, double Kp, double Ki, double Kd, double maxV, double maxA, String name, double[] softLimits) {
      this(motor, tolerance, useAbsoluteEncoder, Ks, Kg, Kv, Kp, Ki, Kd, maxV, maxA, name);
      softLimitMin = softLimits[0];
      softLimitMax = softLimits[1];
    }

    public HazardArm(HazardSparkMax motor, double tolerance, boolean useAbsoluteEncoder, double Ks, double Kg, double Kv, double Kp, double Ki, double Kd, double maxV, double maxA, String name, double[] softLimits, double[] hardLimits) {
      this(motor, tolerance, useAbsoluteEncoder, Ks, Kg, Kv, Kp, Ki, Kd, maxV, maxA, name, softLimits);
      hardLimitMin = hardLimits[0];
      hardLimitMax = hardLimits[1];
    }

    /***
     * Sets the arm setpoint in radians
     * @param value
     */
    public void setpoint(double value) {
        setpoint = MathUtil.clamp(value, softLimitMin, softLimitMax);
        motor.setSetpointPublishingOnly(setpoint);
    }

    /***
     * Sets a new Kg value (coral/no coral)
     * @param Kg
     */
    public void setLoad(double value) {
      dynamicLoadKg = value;
      //Constants.log(name + " load changed from " + dynamicLoadKg + " to " + dynamicLoadKg);
      feedforward.setKg(Kg + dynamicLoadKg);
    }

    public void stop() {
      Constants.log("Arm stopped");
      stop = true;
    }

    public void unstop() {
      Constants.log("Arm reenabled");
      stop = false;
    }

    public boolean atReference() {
      return feedback.atSetpoint();
    }

    public void setBrake(boolean brake) {
      motor.setBrakeMode(brake);
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

    public void periodic() {
      if (stop) {
        motor.set(0);
        return;
      }
      double motorPosition = motor.getPositionRadians();
      if (useAbsoluteEncoder) {
        motorPosition = motor.getAbsPositionRadians();
      }
      if (!manualControl) {
        var currentState = new State(motorPosition, motor.getVelocityRadPS());
        var v = velocityCurve.calculate(0.02, currentState, new State(setpoint, 0));
        var ff = feedforward.calculate(setpoint, v.velocity);
        var fb = feedback.calculate(motorPosition, new State(setpoint, 0));
        SmartDashboard.putNumber(name + " rotation", Math.toDegrees(motorPosition));
        SmartDashboard.putNumber(name + " setpoint", Math.toDegrees(setpoint));
        if (ff + fb < -6 || ff + fb > 6) {
          Constants.log("OVERCORRECT - DISABLE ff: " + ff + " pfb: " + fb + "acc error: " + feedback.getAccumulatedError());
          motor.setControl(0, ControlType.kVoltage);
          velocityFFtarget.set(currentState.velocity);
          voltagePIDOutput.set(0);
        } else {
          motor.setControl(ff + fb, ControlType.kVoltage);
          velocityFFtarget.set(currentState.velocity);
          voltagePIDOutput.set(ff + fb);
        }
        if (motorPosition < hardLimitMin || motorPosition > hardLimitMax) {
          Constants.log(name + "hard limit reached, disabled");
          stop = true;
        }
      }

      motor.publishToNetworkTables();
    }

    public void pushTuningValues() {
      boolean fail = false;
      fail |= !SmartDashboard.putNumber(name + " Ks", feedforward.getKs());
      fail |= !SmartDashboard.putNumber(name + " Kg", feedforward.getKg());
      fail |= !SmartDashboard.putNumber(name + " Kv", feedforward.getKv());

      fail |= !SmartDashboard.putNumber(name + " Kp", feedback.getP());
      fail |= !SmartDashboard.putNumber(name + " Ki", feedback.getI());
      fail |= !SmartDashboard.putNumber(name + " Kd", feedback.getD());

      fail |= !SmartDashboard.putNumber(name + " maxV", feedback.getConstraints().maxVelocity);
      fail |= !SmartDashboard.putNumber(name + " maxA", feedback.getConstraints().maxAcceleration);
      if (fail) { DriverStation.reportError("Failed to push all tuning values to the dashboard!", true); };
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
        this.Kg = Kg;
        feedforward.setKg(Kg + dynamicLoadKg);
        feedforward.setKv(Kv);
        feedback.setPID(Kp, Ki, Kd);
        feedback.setConstraints(new Constraints(maxV, maxA));
    }
}
