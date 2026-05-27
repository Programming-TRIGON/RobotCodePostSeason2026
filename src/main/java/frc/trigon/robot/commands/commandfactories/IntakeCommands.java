package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeCommands {
    public static Command getIntakeCommand() {
        return new StartEndCommand(
                 getSetTargetStateCommand
        )
    }
}
