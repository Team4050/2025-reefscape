// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivetrain drivetrain) {
    return Commands.sequence(new FollowPath(drivetrain));
  }

  public static Command timedMovementTest(Drivetrain drivetrain) {
    return Commands.sequence(
      new MoveTime(drivetrain, 0.2, 0, 0, 0.5),
      new MoveTime(drivetrain, 0, 0.2, 0, 0.5),
      new MoveTime(drivetrain, -0.2, -0.2, 0, 0.5));
  }

  public static Command velTest(Drivetrain drivetrain) {
    return new DVel(drivetrain);
  }

  public static Command OrientLimelight(Drivetrain drivetrain) {
    return new OrientLL(drivetrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
