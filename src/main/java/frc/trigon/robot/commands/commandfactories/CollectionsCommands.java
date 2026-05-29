package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class CollectionsCommands {
    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
        );
    }

    public static Command getReloadCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.PRELOAD),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.PRELOAD)
        );
    }
}