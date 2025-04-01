package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class ChooseAutonomous {
  private List<Command> autos;
  private SendableChooser<Command> autoChooser;

  /*
   * For all reef align commands
   * first couple seconds before auto align should be moving the mechanism
   * time based commands should total up to a max of 5 seconds to leave time for aligning and scoring
   * time based speed should remain below 0.4 (40%), and above 0.1 (10%)
   * There should not be any sudden, jittery movements, if there are, check logs of battery voltage and chassis velocity and estimated pose
   */

  public ChooseAutonomous(Drivetrain drivetrain, Elevator elevator, Claw claw) {
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Do nothing", new MoveTime(drivetrain, 0, 0, 0, 5));
    autoChooser.setDefaultOption(
        "Move out of starting line", new MoveTime(drivetrain, 0.2, 0, 0, 2.0));
    autoChooser.addOption(
        "Align center red",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 4, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new WaitCommand(1),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
    autoChooser.addOption(
        "Align red from left driver side",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 2, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 1.5),
            new MoveTime(drivetrain, 0, 0, 0.2, 1.4),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
    autoChooser.addOption(
        "Align red from right driver side",
        Commands.sequence(
            new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 6, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 1.5),
            new MoveTime(drivetrain, 0, 0, -0.2, 1.8),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
            autoChooser.addOption(
        "Align center blue",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 4, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new WaitCommand(1),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
    autoChooser.addOption(
        "Align blue from left driver side",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(7, 6, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 1.5),
            new MoveTime(drivetrain, 0, 0, 0.2, 1.8),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
    autoChooser.addOption(
        "Align blue from right driver side",
        Commands.sequence(
            new ResetPoseEstimateTo(drivetrain, new Pose2d(7, 2, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 1.5),
            new MoveTime(drivetrain, 0, 0, -0.2, 1.8),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator)));
    SmartDashboard.putData(autoChooser);
  }

  public Command getAuto() {
    return autoChooser.getSelected();
  }
}
