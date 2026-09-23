package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GeneralAutonomousCommands {
    public static Command getAutonomousScoreCommand(double timeout) {
        return new ParallelDeadlineGroup(
                new WaitCommand(timeout),
        )
    }

/*    public static Command getAutonomousDeliveryCommand() {
    }

    public static Command getAutonomousCollectFromNaturalZoneCommand() {
    }*/
}
