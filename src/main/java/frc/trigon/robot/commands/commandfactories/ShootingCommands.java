package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.misc.shootingcalculations.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition FIXED_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB;

    public static Command getShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        GeneralCommands.runWhen(
                                getLoadForShootingCommand(),
                                SHOOTING_CALCULATIONS::isReadyToShoot
                        ),
                        getSetTargetShootingLocationCommand(),
                        getAimForShootingStateCommand(),
                        getAimCommand()
                )
        );
    }

    public static Command getFixedShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForShootingIfReadyToShootCommand(),
                        getAimForFixedShootingStateCommand()
                )
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForDeliveryIfReadyToShootCommand(),
                        getAimForFixedDeliveryStateCommand()
                )
        );
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> setFixedShootingState(targetState));
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(ShootingCommands::updateShootingCalculations);
    }

    private static Command getAimCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getAimCommand(),
                ShooterCommands.getAimCommand()
        );
    }

    private static Command getLoadForShootingIfReadyToShootCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(),
                SHOOTING_CALCULATIONS::isReadyToShoot
        );
    }

    private static Command getLoadForDeliveryIfReadyToShootCommand() {
        return GeneralCommands.runWhen(
                getLoadForDeliveryCommand(),
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

    private static Command getAimForShootingStateCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                OperatorConstants.DRIVER_CONTROLLER::getLeftX,
                OperatorConstants.DRIVER_CONTROLLER::getLeftY,
                () -> new FlippableRotation2d(SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw(), false)
        );
    }

    private static Command getSetTargetShootingLocationCommand() {
        return new InstantCommand(
                () -> SHOOTING_CALCULATIONS.setTargetShootingLocation(getTargetLocation())
        );
    }

    private static Command getAimForFixedShootingStateCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        OperatorConstants.DRIVER_CONTROLLER::getLeftX,
                        OperatorConstants.DRIVER_CONTROLLER::getLeftY,
                        () -> new FlippableRotation2d(FIXED_SHOOTING_STATE.targetState.targetFieldRelativeYaw(), false)
                ),
                HoodCommands.getSetTargetAngleCommand(() -> FIXED_SHOOTING_STATE.targetState.targetPitch()),
                ShooterCommands.getSetTargetVelocityCommand(() -> FIXED_SHOOTING_STATE.targetState.targetShootingVelocityMetersPerSecond())
        );
    }

    private static Command getAimForFixedDeliveryStateCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getSetTargetAngleCommand(() -> CommandConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(() -> CommandConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND)
        );
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    private static ShootingCalculations.TargetLocation getTargetLocation() {
        if (FieldConstants.isRobotInAllianceZone())
            return ShootingCalculations.TargetLocation.HUB;

        if (isRight())
            return ShootingCalculations.TargetLocation.RIGHT_DELIVERY_LOCATION;

        return ShootingCalculations.TargetLocation.LEFT_DELIVERY_LOCATION;
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
        LEFT_CORNER_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                              double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}