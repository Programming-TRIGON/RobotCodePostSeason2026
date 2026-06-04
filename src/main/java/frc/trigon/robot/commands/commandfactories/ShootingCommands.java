package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.misc.shootingcalculations.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.Supplier;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition FIXED_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB;

    public static Command getShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        GeneralCommands.runWhen(
                                getLoadForShootingCommand(),
                                SHOOTING_CALCULATIONS::isReadyToShoot
                        ),
                        getSetTargetShootingLocationCommand(),
                        getAimForShootingStateCommand(),
                        GeneralCommands.getContinuousConditionalCommand(
                                new ParallelCommandGroup(
                                        HoodCommands.getAimAtHubCommand(),
                                        ShooterCommands.getAimAtHubCommand()
                                ),
                                new ParallelCommandGroup(
                                        HoodCommands.getAimForDeliveryCommand(),
                                        ShooterCommands.getAimForDeliveryCommand()
                                ),
                                ShootingCommands::isRobotInAllianceZone
                        )
                )
        );
    }

    public static Command getFixedShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        GeneralCommands.runWhen(
                                getLoadForShootingCommand(),
                                SHOOTING_CALCULATIONS::isReadyToShoot
                        ),
                        getAimForFixedShootingStateCommand(),
                        HoodCommands.getAimAtHubCommand(),
                        ShooterCommands.getAimAtHubCommand()
                )
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        GeneralCommands.runWhen(
                                getLoadForShootingCommand(),
                                SHOOTING_CALCULATIONS::isReadyToShoot
                        ),
                        getSetFixedShootingStateCommand(FixedShootingPosition.FIXED_DELIVERY),
                        getAimForFixedShootingStateCommand(),
                        HoodCommands.getAimAtHubCommand(),
                        ShooterCommands.getAimAtHubCommand()
                )
        );
    }

    public static Command getLoadForShootingCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
        );
    }


    private static Command getAimForShootingStateCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> 0,
                () -> 0,
                () -> new FlippableRotation2d(SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw(), false)
        );
    }

    private static Command getSetTargetShootingLocationCommand() {
        return new InstantCommand(
                () -> SHOOTING_CALCULATIONS.setTargetShootingLocation(ShootingCalculations.TargetLocation.LEFT_DELIVERY_LOCATION)
        );
    }

    private static Command getAimForFixedShootingStateCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> 0,
                () -> 0,
                () -> new FlippableRotation2d(FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw(), false)
        );
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> setFixedShootingState(targetState));
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    private static final Supplier<ShootingCalculations.TargetLocation> TARGET_LOCATION_SUPPLIER = () -> {
        if (isRobotInAllianceZone()) {
            return ShootingCalculations.TargetLocation.HUB;
        }

        if (isRight()) {
            return ShootingCalculations.TargetLocation.RIGHT_DELIVERY_LOCATION;
        }

        return ShootingCalculations.TargetLocation.LEFT_DELIVERY_LOCATION;
    };

    private static boolean isRobotInAllianceZone() {
        return FieldConstants.isRobotInAllianceZone();
    }

    private static boolean isRight() {
        if (Flippable.isRedAlliance())
            return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2;
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static void setFixedShootingState(FixedShootingPosition fixedShootingState) {
        FIXED_SHOOTING_STATE = fixedShootingState;
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        RIGHT_CORNER_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LEFT_CORNER_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        RIGHT_CORNER_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LEFT_CORNER_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        FIXED_DELIVERY(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(50), 6);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                              double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}