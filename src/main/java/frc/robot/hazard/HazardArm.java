package frc.robot.hazard;

import com.revrobotics.spark.SparkBase.ControlType;

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
    private ProfiledPIDController profiledFeedback;
    private double setpoint = 0;
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
    public HazardArm(HazardSparkMax motor, double tolerance, boolean useAbsoluteEncoder, double Ks, double Kg, double Kv, double Kp, double Ki, double Kd, double maxV, double maxA, String name, boolean tuneUsingDashboard) {
        this.motor = motor;
        this.name = name;
        this.useAbsoluteEncoder = useAbsoluteEncoder;
        this.tuneUsingDashboard = tuneUsingDashboard;
        velocityCurve = new TrapezoidProfile(new Constraints(maxV, maxA));
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        feedback = new PIDController(Kp, Ki, Kd);
        feedback.setIntegratorRange(-5, 5);
        feedback.setTolerance(tolerance);

        profiledFeedback = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxV, maxA));
        profiledFeedback.setIntegratorRange(-1.5, 1.5);
        profiledFeedback.setTolerance(tolerance);
        double motorPosition = motor.getPositionRadians();
        if (useAbsoluteEncoder) {
          motorPosition = motor.getAbsPositionRadians();
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
            if (fail) { DriverStation.reportError("Failed to push all tuning values to the dashboard!", true); };
        }

        voltagePIDOutput = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("PID Output").publish();
        velocityFFtarget = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("Velocity feedforward").publish();
    }

    /***
     * Sets the arm setpoint in radians
     * @param value
     */
    public void setpoint(double value) {
        setpoint = value;
        motor.setSetpointPublishingOnly(setpoint);
    }

    /***
     * Sets a new Kg value (coral/no coral)
     * @param Kg
     */
    public void setLoad(double Kg) {
      Constants.log("Wrist load changed from " + feedforward.getKg() + " to " + Kg);
      feedforward.setKg(Kg);
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
      var currentState = new State(motorPosition, motor.getVelocityRadPS());
      var v = velocityCurve.calculate(0.02, currentState, new State(setpoint, 0));
      //Constants.log("Arm FF target velocity: " + v.velocity);
      var ff = feedforward.calculate(setpoint, v.velocity);
      var fb = feedback.calculate(motorPosition, setpoint);
      var pfb = profiledFeedback.calculate(motorPosition, new State(setpoint, 0));
      SmartDashboard.putNumber(name + " rotation", Math.toDegrees(motorPosition));
      SmartDashboard.putNumber(name + " setpoint", Math.toDegrees(setpoint));
      if (ff + pfb < -6 || ff + pfb > 6) {
        Constants.log("OVERCORRECT - DISABLE ff: " + ff + " pfb: " + pfb + "acc error: " + profiledFeedback.getAccumulatedError());
        motor.setControl(0, ControlType.kVoltage);
        voltagePIDOutput.set(0);
      } else {
        motor.setControl(ff + pfb, ControlType.kVoltage);
        voltagePIDOutput.set(ff + pfb);
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
