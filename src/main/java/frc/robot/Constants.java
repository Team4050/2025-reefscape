// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final double moduleGearReduction = 56d / 12d;
    public static final double NEOKvRadsPerV = 49.5324;
    public static final double NEOKvRPSPerV = 473d / 60d;
    public static final double wheelRadiusMeters = 0.0762; // 3in
    public static final RobotConfig mainConfig = new RobotConfig(
      61.235, 
      5.64646, // Current estimate TODO: empirically determine
      new ModuleConfig(Drivetrain.wheelRadiusMeters,
      8, 
      1.0, 
      DCMotor.getKrakenX60(1), 
      80, 
      4), 0.5588);
  }

  public static class Elevator {
    public static final int left = 6;
    public static final int right = 7;
    public static final double elevatorGearboxReduction = 1d / 10d;
    public static final float elevatorMaxExtension = 40f;
    public static final float elevatorMinExtension = 0.5f;

    public static final int elevatorShoulder = 8;
    public static final float shoudlerGearboxReduction = 1f / 21f;
    public static final float shoulderStartingPositionOffset = 0f;

    public static final int elevatorWrist = 9;
    public static final float wristEncoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final float wristGearboxReduction = 1f / 25f;
    public static final float wristStartingPositionOffset = 0f;
    public static final double elevatorGearboxRotationsToHeightMM = 140.178;
    public static final double elevatorFFVoltage = 0.4128;

    public static final int shoulder = 8;
    public static final double shoulderEncoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final double shoulderGearboxReduction = 1d / 21d; // 7:1 * 3:1 gearbox
  }

  public static class Claw {
    public static final int wristMotor = 9;
    public static final double wristGearboxReduction = 1d / 25d;

    public static final int clawMotor = 10;
  }

  public static class Climber {
    public static final int climber = 5;
    public static final double climberGearReduction = (12d / 54d) * (1d / 49d); //Sprocket * gearbox 220.5:1 ratio
  }

  public static void log(Object o) {
    System.out.println(o);
  }

  public static class Sensors {
    public static ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kZ, IMUAxis.kY, IMUAxis.kX);
    public static Vision vision = new Vision();

    public static double getImuYawVelocityRads() {
      return Math.toRadians(imu.getRate());
    }

    public static Rotation3d getImuRotation3d() {
      return new Rotation3d(new Rotation2d(Math.toRadians(imu.getAngle())));
    }

    public static void calibrate() {
      //imu.configCalTime(CalibrationTime._4s);
      //imu.calibrate();
      vision.periodic();
    }
  }
}
