package frc.trigon.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

public class IndexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                IndexerCommands::getSetTargetVelocityCommand,
                false,
                "Debugging/SpindexerTargetVelocityMetersPerSecond"
        );
    }

    public static Command getSetTargetStateCommand(IndexerConstants.IndexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INDEXER.setTargetState(targetState),
                RobotContainer.INDEXER::stop,
                RobotContainer.INDEXER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.INDEXER.setTargetVoltage(targetVelocityMetersPerSecond),
                RobotContainer.INDEXER::stop,
                RobotContainer.INDEXER
        );
    }
}
