package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class MoveScoringMechanismTo {
  public static double algaeRemoveOffsetMM = -35.0;

    public static Command StartingConfig(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
            elevator.setWrist(Constants.Wrist.startingRotationRadians); }, elevator, claw);
    }

    public static Command Transport(Elevator elevator, Claw claw) {
      Constants.log("Moving to trasport config");
      return new InstantCommand(() -> {
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.transport);
      }, elevator, claw);
    }

    public static Command Climbing(Elevator elevator, Claw claw) {
      return new InstantCommand(() -> {

      }, elevator, claw);
    }

    public static Command AlgaeTransport(Elevator elevator, Claw claw) {
      Constants.log("Moving to algae trasport config");
      return new InstantCommand(() -> {
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.algaeTransport);
      }, elevator, claw);
    }

    public static Command L1(Elevator elevator, Claw claw) {
      Constants.log("Going to L1");
        return new InstantCommand(() -> {
            if (claw.algaeMode) {
              Constants.log("Cannot remove algae from L1! Possible misinput?");
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.L1Scoring);
                elevator.setWrist(Constants.Wrist.L1Scoring);
            }
        }, elevator, claw);
    }

    public static Command L2(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
          Constants.log("Going to L2");
            if (claw.algaeMode) {
                elevator.set(Constants.Elevator.L2AlgaeRemoval);
                elevator.setShoulder(Constants.Shoulder.L2AlgaeRemoval);
                elevator.setWrist(Constants.Wrist.L2Scoring);
            } else {
                elevator.set(Constants.Elevator.L2Scoring);
                elevator.setShoulder(Constants.Shoulder.L2Scoring);
                elevator.setWrist(Constants.Wrist.L2Scoring);
            }
        }, elevator, claw);
    }

    public static Command L3(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
          Constants.log("Going to L3");
            if (claw.algaeMode) {
                elevator.set(Constants.Elevator.L3AlgaeRemoval);
                elevator.setShoulder(Constants.Shoulder.L3AlgaeRemoval);
                elevator.setWrist(Constants.Wrist.L3AlgaeRemoval);
            } else {
                elevator.set(Constants.Elevator.L3Scoring);
                elevator.setShoulder(Constants.Shoulder.L3Scoring);
                elevator.setWrist(Constants.Wrist.L3Scoring);
            }
        }, elevator, claw);
    }

    public static Command L4(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
          Constants.log("Going to L4");
            if (claw.algaeMode) {
              Constants.log("Cannot remove algae from L4! Possible misinput?");
            } else {
                elevator.set(Constants.Elevator.L4Scoring);
                elevator.setShoulder(Constants.Shoulder.L4Scoring);
                elevator.setWrist(Constants.Wrist.L4Scoring);
            }
        }, elevator, claw);
    }

    public static Command Processor(Elevator elevator, Claw claw) {
      return new InstantCommand(() -> {
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.transport); }, elevator, claw);
    }

    public static Command FeedingConfig(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator, claw);
    }

    /***
     * For PID tuning
     * @param elevator
     * @param claw
     * @return
     */
    public static Command Horizontal(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator, claw);
    }
}
