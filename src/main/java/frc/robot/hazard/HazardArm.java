package frc.robot.hazard;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class HazardArm {
    private HazardSparkMax motor;
    private ArmFeedforward feedforward;
    private ProfiledPIDController feedback;
    private double setpoint = 0;
    private String name;
    private boolean tuneUsingDashboard = false;

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
    public HazardArm(HazardSparkMax motor, double tolerance, double Ks, double Kg, double Kv, double Kp, double Ki, double Kd, double maxV, double maxA, String name, boolean tuneUsingDashboard) {
        this.motor = motor;
        this.name = name;
        this.tuneUsingDashboard = tuneUsingDashboard;
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        feedback = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxV, maxA));
        feedback.setIntegratorRange(0.05, 0.05);
        feedback.setTolerance(tolerance);

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
    }

    /***
     * Sets the arm setpoint in radians
     * @param value
     */
    public void setpoint(double value) {
        setpoint = value;
        motor.setSetpointPublishingOnly(setpoint);
    }

    public boolean atReference() {
        return feedback.atSetpoint();
    }

    public void periodic() {
        var ff = feedforward.calculate(motor.getPositionRadians(), motor.getVelocityRadPS());
        var fb = feedback.calculate(motor.getPositionRadians(), new State(setpoint, 0));
        motor.setControl(ff + fb, ControlType.kVoltage);
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
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        feedback = new ProfiledPIDController(Kp, Ki, Kd, new Constraints(maxV, maxA));
        feedback.setIntegratorRange(0.05, 0.05);
    }
}
