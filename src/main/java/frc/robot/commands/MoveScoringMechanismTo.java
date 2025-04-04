package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class MoveScoringMechanismTo {
  public static double algaeRemoveOffsetMM = -35.0;

  public static Command StartingConfig(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          elevator.isLoading = false;
          elevator.setScoringLevel(0);
          elevator.set(0);
          elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
          elevator.setWrist(Constants.Wrist.startingRotationRadians);
        },
        elevator);
  }

  public static Command Transport(Elevator elevator) {
    Constants.log("Moving to trasport config");
    return new InstantCommand(
        () -> {
          elevator.isLoading = true;
          elevator.setScoringLevel(0);
          elevator.set(Constants.Elevator.transport);
          elevator.setShoulder(Constants.Shoulder.transport);
          elevator.setWrist(Constants.Wrist.transport);
        },
        elevator);
  }

  @Deprecated
  public static Command Climbing(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          elevator.setScoringLevel(0);
          elevator.set(Constants.Elevator.climb);
          elevator.setShoulder(Constants.Shoulder.climb);
          elevator.setWrist(Constants.Wrist.climb);
        },
        elevator);
  }

  public static Command AlgaeTransport(Elevator elevator, Claw claw) {
    Constants.log("Moving to algae trasport config");
    return new InstantCommand(
        () -> {
          elevator.isLoading = true;
          elevator.setScoringLevel(0);
          elevator.set(Constants.Elevator.algaeTransport);
          elevator.setShoulder(Constants.Shoulder.algaeTransport);
          elevator.setWrist(Constants.Wrist.algaeTransport);
        },
        elevator);
  }

  public static Command AlgaeScoring(Elevator elevator, Claw claw) {
    Constants.log(elevator);
    return new InstantCommand(
        () -> {
          elevator.isLoading = false;
          elevator.setScoringLevel(0);
          elevator.set(Constants.Elevator.transport);
          elevator.setShoulder(Constants.Shoulder.transport);
          elevator.setWrist(Constants.Wrist.algaeScore);
        },
        elevator);
  }

  @Deprecated
  public static Command L1(Elevator elevator, Claw claw) {
    Constants.log("Going to L1");
    return new InstantCommand(
        () -> {
          if (claw.algaeMode) {
            Constants.log("Cannot remove algae from L1! Possible misinput?");
          } else {
            elevator.isLoading = false;
            elevator.setScoringLevel(1);
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.L1Scoring);
            elevator.setWrist(Constants.Wrist.L1Scoring);
          }
        },
        elevator);
  }

  public static Command L2(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          Constants.log("Going to L2");
          elevator.isLoading = false;
          elevator.setScoringLevel(2);
          if (claw.algaeMode) {
            elevator.set(Constants.Elevator.L2AlgaeRemoval);
            elevator.setShoulder(Constants.Shoulder.L2AlgaeRemoval);
            elevator.setWrist(Constants.Wrist.L2AlgaeRemoval);
          } else {
            elevator.set(Constants.Elevator.L2Scoring);
            elevator.setShoulder(Constants.Shoulder.L2Scoring);
            elevator.setWrist(Constants.Wrist.L2Scoring);
          }
        },
        elevator);
  }

  public static Command L3(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          Constants.log("Going to L3");
          elevator.isLoading = false;
          elevator.setScoringLevel(3);
          if (claw.algaeMode) {
            elevator.set(Constants.Elevator.L3AlgaeRemoval);
            elevator.setShoulder(Constants.Shoulder.L3AlgaeRemoval);
            elevator.setWrist(Constants.Wrist.L3AlgaeRemoval);
          } else {
            elevator.set(Constants.Elevator.L3Scoring);
            elevator.setShoulder(Constants.Shoulder.L3Scoring);
            elevator.setWrist(Constants.Wrist.L3Scoring);
          }
        },
        elevator);
  }

  public static Command L4(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          Constants.log("Going to L4");
          elevator.isLoading = false;
          if (claw.algaeMode) {
            Constants.log("Cannot remove algae from L4! Possible misinput?");
          } else {
            elevator.setScoringLevel(4);
            elevator.set(Constants.Elevator.L4Scoring);
            elevator.setShoulder(Constants.Shoulder.L4Scoring);
            elevator.setWrist(Constants.Wrist.L4Scoring);
          }
        },
        elevator);
  }

  public static Command Processor(Elevator elevator, Claw claw) {
    return new InstantCommand(
        () -> {
          elevator.isLoading = true;
          elevator.setScoringLevel(0);
          elevator.set(Constants.Elevator.transport);
          elevator.setShoulder(Constants.Shoulder.transport);
          elevator.setWrist(Constants.Wrist.transport);
        },
        elevator);
  }
}
