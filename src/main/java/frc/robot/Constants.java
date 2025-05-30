// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Pose2d startingPose = new Pose2d(7, 4, new Rotation2d(Math.toRadians(0)));

  /***
   * Returns the alliance side. False for red, true for blue.
   * @return
   */
  public static boolean alliance() {
    return false;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;
  }

  public static class Drivetrain {
    public static final int FL = 1;
    public static final int FR = 2;
    public static final int RL = 3;
    public static final int RR = 4;
    public static final int currentLimit = 60;
    public static final double moduleGearReduction = 56d / 12d;
    public static final double NEOKvRadsPerV = 49.5324;
    public static final double NEOKvRPSPerV = 473d / 60d;
    public static final double wheelRadiusMeters = 0.0762; // 3in
    public static final DCMotor drivetrainMotor = DCMotor.getKrakenX60(1);
    public static final Transform3d robotToCamera =
        new Transform3d(
            0.0352, 0.1778, 0.02, new Rotation3d(new Rotation2d(Math.toRadians(-25.4))));
    public static final Translation2d COMOffsetFromWheelbaseCenter = new Translation2d(0, 0);
    public static final double chassisTopPlateHeightMM = 167.64;
    public static final RobotConfig mainConfig =
        new RobotConfig(
            60, // 61.235,
            5.64646, // Current estimate TODO: empirically determine
            new ModuleConfig(
                Drivetrain.wheelRadiusMeters, 12, 0.6, drivetrainMotor, moduleGearReduction, 60, 4),
            new Translation2d[] {
              new Translation2d(0.2794, 0.2794).minus(COMOffsetFromWheelbaseCenter),
              new Translation2d(0.2794, -0.2794).minus(COMOffsetFromWheelbaseCenter),
              new Translation2d(-0.2794, 0.2794).minus(COMOffsetFromWheelbaseCenter),
              new Translation2d(-0.2794, -0.2794).minus(COMOffsetFromWheelbaseCenter)
            });
  }

  public static class Elevator {
    public static final int back = 6;
    public static final int front = 7;
    public static final int currentLimit = 20;
    public static final double gearboxReduction = 1.0 / 10.0;
    public static final double gearboxRotationsToHeightMM = 140.178;
    public static final double elevatorFFVoltage = 0.4128;
    public static final double startingExtension = 0;
    public static final double hardStopExt = 0.5;
    public static final double minExtensionRotations = 0.02;
    public static final double maxExtensionRotations = 4.4;
    public static final double maxHeightExtensionMM = maxExtensionRotations * gearboxRotationsToHeightMM;
    public static final double baseHeightMM =
        812.800 + Drivetrain.chassisTopPlateHeightMM; // From Elevator - Full Assembly CAD

    /* Elevator PID values */
    public static final double p = 1;
    public static final double i = 0.001;
    public static final double d = 0.1;

    /* Elevator setpoint controls */
    /*
     *  How these numbers work
     * Since the wrist is controlled as an arm, gavity compensation is necessary. The problem
     * is, since the wrist is an unusual shape, its center of mass is offset. On the shoulder,
     * if you drew a line from the axis to the center off mass, it would roughly follow the line
     * of the arm. With the wrist, there is no line to conveniently follow, so I had to estimate
     * it. That's what wristRotationOffset is. If the design of the claw ever changes, that
     * variable changes, and the encoders are set to measure the angle of that.
     *
     * How I recommand changing the values:
     * Start with turning on the elevator tuning mode (boolean on RobotContainer line 52) and deploying
     * Move to the setpoint you want to adjust
     * Make manual adjustments with the joysticks until at desired position
     * Press the Y button to print out the setpoints to the RioLog and dashboard
     * Plug those values in below. For wrist and shoulder, you'll need to convert the printed number
     * to radians, by wrapping the value in Math.toRadians(value).
     * Redeploy and test!
     *
     * If you still have questions or the Y button isn't working ask me on slack
     */
    public static final double transport = 0.9;
    public static final double L1Scoring = 1;
    public static final double L2Scoring = 3.8; // 3.8
    public static final double L2AlgaeRemoval = 1.4;
    public static final double L3Scoring = 3.2;
    public static final double L3AlgaeRemoval = 4.37;//1.145;
    public static final double L4Scoring = 3.9;
    public static final double climb = 2;
  }

  public static class Shoulder {
    public static final int leadCAN = 9;
    public static final int followerCAN = 8;
    public static final int currentLimit = 60;
    public static final DCMotor motor = DCMotor.getNEO(1);
    public static final double encoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final double gearboxReduction = 1.0 / 49.0; // 7:1 * 3:1 gearbox
    public static double encoderOffset = 0.3872078365749783; // Increasing offset will push rotational zero downwards
    public static final double startingRotationRadians =
        Math.toRadians(-78); // -0.3 measured from horizontal
    public static final double shoulderMax = Math.toRadians(90);
    public static final double shoulderHardStopMax = Math.toRadians(160);
    public static final double shoulderMin = Math.toRadians(-82); // -0.24 * 2 * Math.PI
    public static final double shoulderHardStopMin = Math.toRadians(-90);

    public static final double shoulderArmLengthMM = 302;
    public static final double shoulderMotorTorqueNM = 0.45;

    /* Shoulder setpoint controls */
    public static final double transport = Math.toRadians(-80); // TODO: See if reducing from -80 improves loading performance
    public static final double L1Scoring = Math.toRadians(-66.5);
    public static final double L2Scoring = Math.toRadians(-52.5);
    public static final double L2AlgaeRemoval = Math.toRadians(-24);
    public static final double L3Scoring = Math.toRadians(-46); //Math.toRadians(73);
    public static final double L3AlgaeRemoval = L2AlgaeRemoval;//Math.toRadians(64.8);
    public static final double L4Scoring = Math.toRadians(66);
    public static final double climb = Math.toRadians(-55);
  }

  public static class Wrist {
    public static final int CAN = 15;
    public static final int currentLimit = 30;
    public static final double encoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final double gearboxReduction = 1.0 / 40.0;
    public static final double wristRotationOffset = Math.toRadians(-50);
    public static final double startingRotationRadians = Math.toRadians(-31) + wristRotationOffset - Math.toRadians(-78); //Starting rotation relative to shoulder
    public static final double wristMaxShoulderOffsetRadians = Math.toRadians(180);
    public static final double wristMinShoulderOffsetRadians = Math.toRadians(-180);
    public static final double wristMax = Math.toRadians(165) + wristRotationOffset; // +105 degrees
    public static final double wristMin = Math.toRadians(-115) + wristRotationOffset; // -95 degrees

    public static final double chuteExitXOffsetMM = 330; // From Claw - Full Assembly CAD
    public static final double chuteExitYOffsetMM = -222; // From Claw - Full Assembly CAD

    /* Wrist setpoint controls */
    public static final double transport = wristRotationOffset + Math.toRadians(-30) - Math.toRadians(-80);
    public static final double algaeTransport = wristRotationOffset + Math.toRadians(26) - Math.toRadians(-80);
    public static final double L1Scoring = startingRotationRadians - Math.toRadians(-66.5);
    public static final double L2Scoring = Math.toRadians(-19) + wristRotationOffset - Math.toRadians(-52.5);
    public static final double L2AlgaeRemoval = Math.toRadians(-20) + wristRotationOffset - Math.toRadians(-24);
    public static final double L3Scoring = wristMax;
    public static final double L3AlgaeRemoval = L2AlgaeRemoval;//Math.toRadians(-10) + wristRotationOffset - Math.toRadians(64.8);
    public static final double L4Scoring = Math.toRadians(130) + wristRotationOffset - Math.toRadians(66);
    public static final double climb = startingRotationRadians - Math.toRadians(-55);

    public static final double scoringOffsetMeters = 1.75 * (2.54 / 100); //TODO: test if 1/4in offset is that important
  }

  public static class Coral {
    public static final int CAN = 10;
    public static final int currentLimit = 10;
    public static final double Yoffset = 0;

    public static final double coralSpeed = 0.5;
    public static final double coralScoreSpeed = 1;
    public static final double algaeSpeed = 0.8;
  }

  public static class Climber {
    public static final int climber = 5;
    public static final int currentLimit = 60;
    public static final double climberGearReduction =
        (12.0 / 54.0) * (1.0 / 49.0); // Sprocket * gearbox 220.5:1 ratio

    public static final double startingPositionRotations = 0;
    public static final double deployedPositionRotations = -0.2; // -0.26389;
    public static final double funnelFoldThresholdRotations = -0.15;
    public static final double climbedPositionRotations = 0.11944;
  }

  public static DataLog matchLog;
  public static DataLogEntry elevatorSysIDEntry;
  public static DoubleArrayLogEntry limelightDataLogEntry;
  public static StringLogEntry eventsEntry;

  public static void setupDataLog() {
    matchLog = DataLogManager.getLog();
    Constants.log("Setting up data log " + DataLogManager.getLogDir());
    DataLogManager.logNetworkTables(true);
    //elevatorSysIDEntry = new DataLogEntry();
    //estimatedPositionLogEntry = new DoubleArrayLogEntry(matchLog, "EstPosition");
    //limelightDataLogEntry = new DoubleArrayLogEntry(matchLog, "Limelight");
    //eventsEntry = new StringLogEntry(matchLog, "Events");
  }

  public static void flushLog() {
    matchLog.flush();
  }

  public static void logStringToFile(String str) {
    eventsEntry.append(str);
  }

  public static void log(Object o) {
    System.out.println(o);
  }

  public static boolean doLogAuto = true;

  public static void autoLog(Object o) {
    if (!doLogAuto) return;
    System.out.println(o);
  }

  public static boolean doLogTeleop = true;

  public static void driverLog(Object o) {
    if (!doLogTeleop) return;
    System.out.println(o);
  }

  public static class Sensors {
    public static ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, IMUAxis.kY, IMUAxis.kX);

    // public static Vision vision = new Vision();

    public static void resetIMU() {
      Constants.log("Resetting gyro angle to 0...");
      imu.reset();
    }

    /***
     * Set IMU angle in degrees
     * @param angle
     */
    public static void setIMUGyroYaw(double angle) {
      Constants.log("Setting gyro angle to " + angle);
      imu.setGyroAngleZ(angle);
    }

    public static double getIMUYawVelocityRads() {
      return Math.toRadians(-imu.getRate());
    }

    public static double getIMUYawRadians() {
      return Math.toRadians(-imu.getAngle());
    }

    public static Rotation3d getImuRotation3d() {
      return new Rotation3d(new Rotation2d(Math.toRadians(-imu.getAngle())));
    }

    public static Rotation2d getImuRotation2d() {
      return new Rotation2d(Math.toRadians(-imu.getAngle()));
    }

    public static double[] getImuAccelXY() {
      return new double[] {imu.getAccelY(), imu.getAccelX()};
    }

    public static void calibrate(Pose3d startingPose) {
      // Current orientation imu is CW+
      imu.configCalTime(CalibrationTime._4s);
      imu.calibrate();
      imu.reset();
      imu.setGyroAngleZ(Math.toDegrees(startingPose.getRotation().getAngle()));
      Constants.log("IMU starting angle: " + imu.getAngle());
    }
  }
}

/*
 * Shoulder PID values: In elevator file, line 206
 * Wrist PID values: In elevator file, line 222
 *
 * Auto score command is in AutoScore.java
 * Auto load command is in AutoLoad.java
 * Auto align command is AutoAlignPIDVoltage.java
 * Auto align PID values start line 28
 */