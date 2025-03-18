// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

@Deprecated
public final class ReturnAutonomous {
  /** Example static factory for an autonomous command. */
  public static Command moveOut(Drivetrain drivetrain) {
    return Commands.sequence(new MoveTime(drivetrain, 0.2, 0, 0, 0.8));
  }

  public static Command testModelBasedControl(Drivetrain drivetrain) {
    return new TestModelBasedControl(drivetrain);
  }

  public static Command testModelBasedPathFollowing(Drivetrain drivetrain) {
    return new FollowPath(drivetrain);
  }

  public static Command OrientLimelight(Drivetrain drivetrain) {
    return new AlignToReefPID(drivetrain, false, true);
  }

  private ReturnAutonomous() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
