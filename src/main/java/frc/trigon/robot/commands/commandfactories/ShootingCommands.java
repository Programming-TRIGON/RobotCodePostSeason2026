package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
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

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition FIXED_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB;

    public static Command getShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForShootingWhenReadyCommand(),
                        getSetTargetShootingLocationCommand(),
                        getAimForShootingStateCommand(),
                        getAimForShootingCommand()
                )
        );
    }

    public static Command getFixedShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        GeneralCommands.runWhen(
                                getLoadForShootingCommand(),
                                ShootingCommands::isReadyForFixedShooting
                        ),
                        getAimForFixedShootingCommand()
                )
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        GeneralCommands.runWhen(
                                getLoadForDeliveryCommand(),
                                ShootingCommands::isReadyForFixedDelivery
                        ),
                        getAimForFixedDeliveryCommand()
                )
        );
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> setFixedShootingState(targetState));
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

    private static Command getLoadForShootingWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(),
                SHOOTING_CALCULATIONS::isReadyToShoot
        );
    }

    private static Command getLoadForDeliveryCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_DELIVERY),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_DELIVERY)
        );
    }

    private static Command getAimForShootingStateCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> new FlippableRotation2d(SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw(), false)
        );
    }

    private static Command getSetTargetShootingLocationCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> SHOOTING_CALCULATIONS.setTargetShootingLocation(getTargetLocation())),
                new InstantCommand(),
                () -> getTargetLocation() != SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation()
        ).repeatedly();
    }

    private static Command getAimForFixedShootingCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw(), false)
                ),
                HoodCommands.getSetTargetAngleCommand(FIXED_SHOOTING_STATE.targetState.targetPitch()),
                ShooterCommands.getSetTargetVelocityCommand(FIXED_SHOOTING_STATE.targetState.targetShootingVelocityMetersPerSecond())
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
        final boolean isYawReady = RobotContainer.SWERVE.atAngle(new FlippableRotation2d(FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw(), false));
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(FIXED_SHOOTING_STATE.targetState.targetPitch());
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(FIXED_SHOOTING_STATE.targetState.targetShootingVelocityMetersPerSecond());

        return isYawReady && isPitchReady && isVelocityReady;
    }

    private static Command getLoadForShootingCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
        );
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    private static ShootingCalculations.TargetLocation getTargetLocation() {
        if (FieldConstants.isRobotInAllianceZone())
            return ShootingCalculations.TargetLocation.HUB;

        if (FieldConstants.isRight())
            return ShootingCalculations.TargetLocation.RIGHT_DELIVERY_LOCATION;

        return ShootingCalculations.TargetLocation.LEFT_DELIVERY_LOCATION;
    }

    private static void setFixedShootingState(FixedShootingPosition fixedShootingState) {
        FIXED_SHOOTING_STATE = fixedShootingState;
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(45), 5),
        RIGHT_CORNER_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(34), 10),
        LEFT_CORNER_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(23), 8),
        RIGHT_CORNER_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(12), 9),
        LEFT_CORNER_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(23), 6);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch, double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}