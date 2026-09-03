package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.hood.HoodConstants;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.kicker.KickerCommands;
import frc.trigon.robot.subsystems.kicker.KickerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

import java.util.function.BooleanSupplier;

public class EjectionCommands {
    public static Command getEjectFromIntakeCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.EJECT_FROM_INTAKE),
                KickerCommands.getSetTargetStateCommand(KickerConstants.KickerState.EJECT_FROM_INTAKE),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.EJECT_FROM_INTAKE),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REVERSE_POWERED_OPEN)
        );
    }

    public static Command getEjectFromShooterCommand() {
        return new ParallelCommandGroup(
                getLoadForEjectFromShooterWhenReadyCommand(),
                ShooterCommands.getSetTargetVelocityCommand(() -> ShooterConstants.EJECT_FROM_SHOOTER_TARGET_VELOCITY_METERS_PER_SECOND),
                HoodCommands.getSetTargetAngleCommand(() -> HoodConstants.EJECT_FROM_SHOOTER_PITCH),
                ShootingCommands.getIntakeSequenceWhileShootingCommand());
    }

    private static Command getLoadForEjectFromShooterWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForEjectFromShooterCommand(),
                isReadyToEjectFromShooter()
        );
    }

    private static ParallelCommandGroup getLoadForEjectFromShooterCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.EJECT_FROM_SHOOTER),
                KickerCommands.getSetTargetStateCommand(KickerConstants.KickerState.EJECT_FROM_SHOOTER),
                LoaderCommands.getSetTargetVelocityCommand(() -> LoaderConstants.EJECT_FROM_SHOOTER_VELOCITY)
        );
    }

    private static BooleanSupplier isReadyToEjectFromShooter() {
        return () -> RobotContainer.SHOOTER.atVelocity(ShooterConstants.EJECT_FROM_SHOOTER_TARGET_VELOCITY_METERS_PER_SECOND)
                && RobotContainer.HOOD.atAngle(HoodConstants.EJECT_FROM_SHOOTER_PITCH);
    }
}
