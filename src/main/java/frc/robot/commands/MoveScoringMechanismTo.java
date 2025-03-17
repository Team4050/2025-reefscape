package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class MoveScoringMechanismTo {
  public static double algaeRemoveOffsetMM = -35.0;

    public static Command StartingConfig(Elevator elevator) {
        return new InstantCommand(() -> {
          Constants.log("Moving to starting config");
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotationRadians);
            elevator.setWrist(Constants.Wrist.startingRotationRadians); }, elevator);
    }

    public static Command Transport(Elevator elevator) {
      return new InstantCommand(() -> {
        Constants.log("Moving to trasport config");
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.transport);
      }, elevator);
    }

    public static Command Climbing(Elevator elevator) {
      return new InstantCommand(() -> {
        Constants.log("Moving to climbing config");
        elevator.set(Constants.Elevator.climb);
        elevator.setShoulder(Constants.Shoulder.climb);
        elevator.setWrist(Constants.Wrist.climb);
      }, elevator);
    }

    public static Command AlgaeTransport(Elevator elevator) {
      return new InstantCommand(() -> {
        Constants.log("Moving to algae trasport config");
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.algaeTransport);
      }, elevator);
    }

    public static Command L1(Elevator elevator) {

        return new InstantCommand(() -> {
          Constants.log("Moving to L1 scoring config");
            if (elevator.algaeMode) {
              Constants.log("Cannot remove algae from L1! Possible misinput?");
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.L1Scoring);
                elevator.setWrist(Constants.Wrist.L1Scoring);
            }
        }, elevator);
    }

    public static Command L2(Elevator elevator) {
        return new InstantCommand(() -> {
          Constants.log("Moving to L2 scoring config");
            if (elevator.algaeMode) {
                elevator.set(Constants.Elevator.L2AlgaeRemoval);
                elevator.setShoulder(Constants.Shoulder.L2AlgaeRemoval);
                elevator.setWrist(Constants.Wrist.L2Scoring);
            } else {
                elevator.set(Constants.Elevator.L2Scoring);
                elevator.setShoulder(Constants.Shoulder.L2Scoring);
                elevator.setWrist(Constants.Wrist.L2Scoring);
            }
        }, elevator);
    }

    public static Command L3(Elevator elevator) {
        return new InstantCommand(() -> {
          Constants.log("Moving to L3 scoring config");
            if (elevator.algaeMode) {
                elevator.set(Constants.Elevator.L3AlgaeRemoval);
                elevator.setShoulder(Constants.Shoulder.L3AlgaeRemoval);
                elevator.setWrist(Constants.Wrist.L3AlgaeRemoval);
            } else {
                elevator.set(Constants.Elevator.L3Scoring);
                elevator.setShoulder(Constants.Shoulder.L3Scoring);
                elevator.setWrist(Constants.Wrist.L3Scoring);
            }
        }, elevator);
    }

    public static Command L4(Elevator elevator) {
        return new InstantCommand(() -> {
          Constants.log("Moving to L4 scoring config");
            if (elevator.algaeMode) {
              Constants.log("Cannot remove algae from L4! Possible misinput?");
            } else {
                elevator.set(Constants.Elevator.L4Scoring);
                elevator.setShoulder(Constants.Shoulder.L4Scoring);
                elevator.setWrist(Constants.Wrist.L4Scoring);
            }
        }, elevator);
    }

    public static Command Processor(Elevator elevator) {
      return new InstantCommand(() -> {
        Constants.log("Moving to processor scoring config");
        elevator.set(Constants.Elevator.transport);
        elevator.setShoulder(Constants.Shoulder.transport);
        elevator.setWrist(Constants.Wrist.transport); }, elevator);
    }

    /***
     * For PID tuning
     * @param elevator
     * @param claw
     * @return
     */
    public static Command Horizontal(Elevator elevator) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(0);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator);
    }
}
