package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class ChooseAutonomous {
    private List<Command> autos;
    private SendableChooser<Command> autoChooser;

    public ChooseAutonomous(Drivetrain drivetrain, Elevator elevator, Claw claw) {
        NamedCommands.registerCommand("Score L4", MoveScoringMechanismTo.L4(elevator, claw));
        Command startingLineToTag = AutoBuilder.buildAuto("Move to tag");
        Command crossedFingers = AutoBuilder.buildAuto("Crossed fingers");
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Move out of starting line", new MoveTime(drivetrain, 0, 0, 0, 0.5));
        autoChooser.addOption("Time-based test", Commands.sequence(
            new MoveTime(drivetrain, 0.2, 0, 0, 0.5),
            new MoveTime(drivetrain, 0, 0.2, 0, 0.5),
            new MoveTime(drivetrain, -0.2, -0.2, 0, 0.5)));
        autoChooser.addOption("Model-based control test", new TestModelBasedControl(drivetrain));
        autoChooser.addOption("Teleop Limelight align", new AlignToReefTest(drivetrain, false, false));
        autoChooser.addOption("Crossed fingers", crossedFingers);
        autoChooser.addOption("Starting line to tag", startingLineToTag);
        SmartDashboard.putData(autoChooser);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }
}
