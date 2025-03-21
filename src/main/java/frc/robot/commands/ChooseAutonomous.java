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

  public ChooseAutonomous(Drivetrain drivetrain, Elevator elevator, Claw claw) {
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Do nothing", new MoveTime(drivetrain, 0, 0, 0, 5));
    autoChooser.setDefaultOption(
        "Move out of starting line", new MoveTime(drivetrain, 0.2, 0, 0, 0.5));
    autoChooser.addOption(
        "Align center red",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 4, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new WaitCommand(0.75),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator, claw)));
    autoChooser.addOption(
        "Align red from left driver side",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 2, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 0.8),
            new MoveTime(drivetrain, 0, 0, 0.2, 0.4),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5)));
    autoChooser.addOption(
        "Align red from right driver side",
        Commands.sequence(
            new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 6, Rotation2d.kZero)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 0.8),
            new MoveTime(drivetrain, 0, 0, -0.2, 0.4),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator, claw)));
            autoChooser.addOption(
        "Align center blue",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(10, 4, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new WaitCommand(0.75),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator, claw)));
    autoChooser.addOption(
        "Align blue from left driver side",
        Commands.sequence(
          new ResetPoseEstimateTo(drivetrain, new Pose2d(7, 6, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 0.8),
            new MoveTime(drivetrain, 0, 0, 0.2, -0.4),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5)));
    autoChooser.addOption(
        "Align blue from right driver side",
        Commands.sequence(
            new ResetPoseEstimateTo(drivetrain, new Pose2d(7, 2, Rotation2d.k180deg)),
            MoveScoringMechanismTo.L4(elevator, claw),
            new MoveTime(drivetrain, 0.2, 0, 0, 0.8),
            new MoveTime(drivetrain, 0, 0, -0.2, 0.4),
            new AlignToReefPIDVoltage(drivetrain, false, false),
            new AutoScore(elevator, claw),
            new MoveTime(drivetrain, -0.2, 0, 0, 0.5),
            MoveScoringMechanismTo.Transport(elevator, claw)));
    SmartDashboard.putData(autoChooser);
  }

  public Command getAuto() {
    return autoChooser.getSelected();
  }
}
