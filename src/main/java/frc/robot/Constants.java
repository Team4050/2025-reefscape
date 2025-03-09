// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.Vision;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    public static final RobotConfig mainConfig = new RobotConfig(
      61.235, 
      5.64646, // Current estimate TODO: empirically determine
      new ModuleConfig(Drivetrain.wheelRadiusMeters,
      8, 
      1.0, 
      drivetrainMotor, 
      80, 
      4), 0.5588);
    public static final Transform3d robotToCamera = new Transform3d(0.185, 0, 0.27, new Rotation3d());
    public static final Translation2d COMOffsetFromWheelbaseCenter = new Translation2d(0, 0);
    public static final double chassisTopPlateHeightMM = 167.64;
  }

  public static class Elevator {
    public static final int back = 6;
    public static final int front = 7;
    public static final int elevatorCurrentLimit = 20;
    public static final double gearboxReduction = 1.0 / 10.0;
    public static final double gearboxRotationsToHeightMM = 140.178;
    public static final double elevatorFFVoltage = 0.4128;
    public static final double minExtension = 0.02;
    public static final double maxExtension = 4.4;
    public static final double maxHeightExtensionMM = maxExtension * gearboxRotationsToHeightMM;
    public static final double baseHeightMM = 812.800 + Drivetrain.chassisTopPlateHeightMM; //From Elevator - Full Assembly CAD

    public static final double L1Scoring = 0;
    public static final double L2Scoring = 2.77; //~2.77
    public static final double L3Scoring = 2.34;
    public static final double L4Scoring = 0;
  }

  public static class Shoulder {
    public static final int CAN = 8;
    public static final int currentLimit = 40;
    public static final DCMotor motor = DCMotor.getNEO(1);
    public static final double encoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final double gearboxReduction = 1.0 / 49.0; // 7:1 * 3:1 gearbox
    public static final double startingRotation = -0.25; // -0.23 TODO: find out
    public static final double shoulderMax = 0.25; //0.46 rotations
    public static final double shoulderMin = -0.24;

    public static final double shoulderArmLengthMM = 302;
    public static final double shoulderMotorTorqueNM = 0.45;

    public static final double L1Scoring = 0;
    public static final double L2Scoring = -0.22; //~0.22
    public static final double L3Scoring = 0.22;
    public static final double L4Scoring = 0.25;
  }

  public static class Wrist {
    public static final int CAN = 9;
    public static final int currentLimit = 30;
    public static final double encoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final double gearboxReduction = 1.0 / 25.0;
    public static final double startingRotation = -0.1055555;
    public static final double wristMaxShoulderOffsetRotations = 0.5;
    public static final double wristMinShoulderOffsetRotations = -0.35;
    public static final double wristMax = 0.3;//+108 degrees
    public static final double wristMin = -0.125; //-45 degrees

    public static final double chuteCenterXOffsetMM = 330; //From Claw - Full Assembly CAD
    public static final double chuteCenterYOffsetMM = -222; //From Claw - Full Assembly CAD

    public static final double L1Scoring = 0;
    public static final double L2Scoring = startingRotation;
    public static final double L3Scoring = startingRotation;
    public static final double L4Scoring = 0.25;
  }

  public static class Coral {
    public static final int CAN = 10;
    public static final int currentLimit = 20;
    public static final double clawGearboxReduction = 1.0 / 10.0;
    public static final double Yoffset = 0;
  }

  public static class Climber {
    public static final int climber = 5;
    public static final int currentLimit = 40;
    public static final double climberGearReduction = (12.0 / 54.0) * (1.0 / 49.0); //Sprocket * gearbox 220.5:1 ratio

    public static final double startingPosition = 0;
    public static final double deployedPosition = -0.5;
    public static final double climbedPosition = 0.25;
  }

  public static void log(Object o) {
    System.out.println(o);
  }

  public static class Sensors {
    public static ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, IMUAxis.kY, IMUAxis.kX);
    //public static Vision vision = new Vision();

    public static void resetIMU() {
      Constants.log("Resetting gyro angle to 0...");
      imu.setGyroAngleZ(0);
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
      //Current orientation imu is CW+
      imu.configCalTime(CalibrationTime._4s);
      imu.calibrate();
      imu.setGyroAngleZ(180);
      //vision.setRobotPose(startingPose);
      //vision.setPipelineIndex(0);
    }
  }
}