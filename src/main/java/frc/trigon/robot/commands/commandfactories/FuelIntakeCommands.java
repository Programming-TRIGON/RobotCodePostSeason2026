package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;

public class FuelIntakeCommands {
    public static Command getPreloadCommand() {
        return new ParallelCommandGroup(
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.PRELOAD)
        ).withTimeout(CommandConstants.PRELOAD_TIMER_SECONDS);
    }

    public static Command getCloseIntakeWhileShootingCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE),
                new InstantCommand(() -> CommandConstants.setSpeedMultiplier(CommandConstants.CLOSE_INTAKE_SWERVE_SPEED_MULTIPLIER))
        ).finallyDo(() -> CommandConstants.setSpeedMultiplier(CommandConstants.DEFAULT_SWERVE_SPEED_MULTIPLIER)).until(OperatorConstants.IS_ANY_SHOOTING_TRIGGER_ACTIVE.negate());
    }
}