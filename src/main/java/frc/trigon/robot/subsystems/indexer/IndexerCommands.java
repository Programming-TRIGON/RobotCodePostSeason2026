package frc.trigon.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

public class IndexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVoltage) -> new StartEndCommand(
                        () -> RobotContainer.INDEXER.setTargetVoltage(targetVoltage),
                        RobotContainer.INDEXER::stop,
                        RobotContainer.INDEXER
                ),
                false,
                "Debugging/IndexerTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IndexerConstants.IndexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INDEXER.setTargetState(targetState),
                RobotContainer.INDEXER::stop,
                RobotContainer.INDEXER
        );
    }
}
