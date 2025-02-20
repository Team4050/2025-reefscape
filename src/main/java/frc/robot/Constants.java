// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class Drivetrain {
    public static final int FL = 1;
    public static final int FR = 2;
    public static final int RL = 3;
    public static final int RR = 4;
    public static final float moduleGearReduction = 56 / 12;
    public static final float wheelRadiusMeters = 0.0762f; // 3in
  }

  public static class Elevator {
    public static final int elevatorLeft = 5;
    public static final int elevatorRight = 6;
    public static final int elevatorWrist = 7;
    public static final int elevatorGearboxReduction = 1 / 25;
    public static final int wristEncoderCountsPerRevolution = 4096; // CTRE Mag counts per rev
    public static final int wristGearboxReduction = 21; // 7:1 * 3:1 gearbox
    public static final double elevatorGearboxRotationsToHeightMM = 140.178;
    public static final double elevatorFFVoltage = 0.4128;
  }

  public static class Claw {
    public static final int clawMotor = 8;
  }

  public static void log(Object o) {
    System.out.println(o);
  }
}
