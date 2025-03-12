package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class MoveScoringMechanismTo {
    public static Command StartingConfig(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotation);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator, claw);
    }

    public static Command L1(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            if (claw.algaeMode) {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            }
        }, elevator, claw);
    }

    public static Command L2(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            if (claw.algaeMode) {
                elevator.set(Constants.Elevator.L2Scoring);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            }
        }, elevator, claw);
    }

    public static Command L3(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            if (claw.algaeMode) {
                elevator.set(Constants.Elevator.L3Scoring);
                elevator.setShoulder(Constants.Shoulder.L3Scoring);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.L3Scoring);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            }
        }, elevator, claw);
    }

    public static Command L4(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            if (claw.algaeMode) {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            } else {
                elevator.set(0);
                elevator.setShoulder(Constants.Shoulder.startingRotation);
                elevator.setWrist(Constants.Wrist.startingRotationRadians);
            }
        }, elevator, claw);
    }

    public static Command Processor(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotation);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator, claw);
    }

    public static Command FeedingConfig(Elevator elevator, Claw claw) {
        return new InstantCommand(() -> {
            elevator.set(0);
            elevator.setShoulder(Constants.Shoulder.startingRotation);
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
            elevator.setShoulder(Constants.Shoulder.startingRotation);
            elevator.setWrist(Constants.Wrist.startingRotationRadians);}, elevator, claw);
    }
}
