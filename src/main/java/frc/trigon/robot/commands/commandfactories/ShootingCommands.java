package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.misc.shootingcalculations.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.hood.HoodConstants;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.Supplier;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition TARGET_FIXED_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB;

    public static Command getCalibrateShootingCalculationsCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getDebuggingCommand(),
                ShooterCommands.getDebuggingCommand(),
                GeneralCommands.runWhen(
                        new ParallelCommandGroup(
                                IndexerCommands.getDebuggingCommand(),
                                LoaderCommands.getDebuggingCommand()
                        ),
                        () -> RobotContainer.HOOD.atTargetAngle() && RobotContainer.SHOOTER.atTargetVelocity()
                )
        );
    }

    public static Command getShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForShootingWhenReadyCommand(),
                        getSetTargetShootingLocationCommand(),
                        getAimSwerveCommand(() -> SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw()),
                        getAimForShootingCommand()
                )
        );
    }

    public static Command getFixedShootingAtHubCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getLoadForFixedShootingWhenReadyCommand(),
                        getAimSwerveCommand(() -> TARGET_FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw()),
                        HoodCommands.getSetTargetAngleCommand(TARGET_FIXED_SHOOTING_STATE.targetState.targetPitch()),
                        ShooterCommands.getSetTargetVelocityCommand(TARGET_FIXED_SHOOTING_STATE.targetState.targetShootingVelocityMetersPerSecond())
                )
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getLoadForDeliveryWhenReadyCommand(),
                        getAimForFixedDeliveryCommand()
                )
        );
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> setTargetFixedShootingState(targetState));
    }

    private static Command getLoadForFixedShootingWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(),
                ShootingCommands::isReadyForFixedShooting
        );
    }

    private static Command getLoadForDeliveryWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForDeliveryCommand(),
                ShootingCommands::isReadyForFixedDelivery
        );
    }

    private static Command getLoadForShootingWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(),
                SHOOTING_CALCULATIONS::isReadyToShoot
        );
    }

    private static Command getLoadForShootingCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
        );
    }

    private static Command getLoadForDeliveryCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_DELIVERY),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_DELIVERY)
        );
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(ShootingCommands::updateShootingCalculations);
    }

    private static Command getAimForShootingCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getAimForShootingCommand(),
                ShooterCommands.getAimForShootingCommand()
        );
    }

    private static Command getAimSwerveCommand(Supplier<Rotation2d> rotationSupplier) {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> new FlippableRotation2d(rotationSupplier.get(), false)
        );
    }

    private static Command getSetTargetShootingLocationCommand() {
        return GeneralCommands.runWhen(
                new InstantCommand(() -> SHOOTING_CALCULATIONS.setTargetShootingLocation(getTargetLocation())),
                () -> getTargetLocation() != SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation()
        );
    }

    private static Command getAimForFixedDeliveryCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getSetTargetAngleCommand(HoodConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShooterConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND)
        );
    }

    private static boolean isReadyForFixedDelivery() {
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(HoodConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH);
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(ShooterConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND);

        return isPitchReady && isVelocityReady;
    }

    private static boolean isReadyForFixedShooting() {
        final boolean isYawReady = RobotContainer.SWERVE.atAngle(new FlippableRotation2d(TARGET_FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw(), false));
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(TARGET_FIXED_SHOOTING_STATE.targetState.targetPitch());
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(TARGET_FIXED_SHOOTING_STATE.targetState.targetShootingVelocityMetersPerSecond());

        return isYawReady && isPitchReady && isVelocityReady;
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    private static ShootingCalculations.TargetShootingLocation getTargetLocation() {
        if (FieldConstants.isRobotInAllianceZone())
            return ShootingCalculations.TargetShootingLocation.HUB;

        if (FieldConstants.isRight())
            return ShootingCalculations.TargetShootingLocation.RIGHT_DELIVERY_LOCATION;

        return ShootingCalculations.TargetShootingLocation.LEFT_DELIVERY_LOCATION;
    }

    private static void setTargetFixedShootingState(FixedShootingPosition targetFixedShootingState) {
        TARGET_FIXED_SHOOTING_STATE = targetFixedShootingState;
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(45), 5),
        RIGHT_TRENCH(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(34), 10),
        LEFT_TRENCH(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(23), 8),
        BACK_RIGHT(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(29), 9),
        BACK_LEFT(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(23), 6);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch, double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}