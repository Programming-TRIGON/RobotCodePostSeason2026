package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.hood.HoodConstants;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition TARGET_FIXED_SHOOTING_AT_HUB_STATE = FixedShootingPosition.IN_FRONT_OF_TOWER;

    public static Command getShootingMapCalibrationCommand() {
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
                        getLoadForShootingWhenReadyCommand(() -> SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation().isDelivery),
                        getSetTargetShootingLocationCommand(),
                        getAimSwerveCommand(() -> SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw()),
                        getAimForShootingCommand(),
                        getIntakeSequenceWhileShootingCommand()
                )
        );
    }

    public static Command getAutonomousShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForShootingWhenReadyCommand(() -> SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation().isDelivery),
                        getSetTargetShootingLocationCommand(),
                        getAimSwerveCommand(() -> SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw()),
                        getAimForShootingCommand(),
                        IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE)
                )
        );
    }

    public static Command getFixedShootingAtHubCommand() {
        return new ParallelCommandGroup(
                getLoadForFixedShootingAtHubWhenReadyCommand(),
                HoodCommands.getSetTargetAngleCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetPitch),
                ShooterCommands.getSetTargetVelocityCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetShootingVelocityMetersPerSecond),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", TARGET_FIXED_SHOOTING_AT_HUB_STATE.name())),
                getIntakeSequenceWhileShootingCommand()
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new ParallelCommandGroup(
                getLoadForFixedDeliveryWhenReadyCommand(),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/isReadyForFixedDelivery", isReadyForFixedDelivery())),
                HoodCommands.getSetTargetAngleCommand(() -> HoodConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(() -> ShooterConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND),
                getIntakeSequenceWhileShootingCommand()
        );
    }

    private static RepeatCommand getIntakeSequenceWhileShootingCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).until(OperatorConstants.CLOSE_INTAKE_WHILE_SHOOTING_TRIGGER),
                FuelIntakeCommands.getCloseIntakeWhileShootingCommand().until(OperatorConstants.INTAKE_WHILE_SHOOTING_TRIGGER)
        ).repeatedly();
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> {
            setTargetFixedShootingAtHubState(targetState);
            Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", targetState.name());
        });
    }

    private static Command getLoadForFixedShootingAtHubWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(() -> false),
                ShootingCommands::isReadyForFixedShootingAtHub
        );
    }

    private static Command getLoadForFixedDeliveryWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(() -> true),
                ShootingCommands::isReadyForFixedDelivery
        );
    }

    private static Command getLoadForShootingWhenReadyCommand(BooleanSupplier isDelivery) {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(isDelivery).until(() -> !SHOOTING_CALCULATIONS.isReadyToShoot()),
                SHOOTING_CALCULATIONS::isReadyToShoot
        ).repeatedly();
    }

    private static Command getLoadForShootingCommand(BooleanSupplier isDelivery) {
        return GeneralCommands.getContinuousConditionalCommand(
                getLoadForDeliveryCommand(),
                getLoadForShootingAtHubCommand(),
                isDelivery
        );
    }

    private static ParallelCommandGroup getLoadForDeliveryCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_DELIVERY),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_DELIVERY)
        );
    }

    private static ParallelCommandGroup getLoadForShootingAtHubCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
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
        return new RunCommand(() -> {
            final ShootingCalculations.TargetShootingLocation target = getTargetLocation();
            if (target != SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation())
                SHOOTING_CALCULATIONS.setTargetShootingLocation(target);
        });
    }

    private static boolean isReadyForFixedDelivery() {
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(HoodConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH);
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(ShooterConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND);

        return isPitchReady && isVelocityReady;
    }

    private static boolean isReadyForFixedShootingAtHub() {
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetPitch);
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetShootingVelocityMetersPerSecond);

        return isPitchReady && isVelocityReady;
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

    private static void setTargetFixedShootingAtHubState(FixedShootingPosition targetFixedShootingAtHubState) {
        TARGET_FIXED_SHOOTING_AT_HUB_STATE = targetFixedShootingAtHubState;
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations IRL
        IN_FRONT_OF_TOWER(Rotation2d.fromDegrees(59.965), 6.866),
        RIGHT_TRENCH(Rotation2d.fromDegrees(59.427), 7.108),
        LEFT_TRENCH(Rotation2d.fromDegrees(59.427), 7.108),
        BACK_RIGHT(Rotation2d.fromDegrees(57.079), 8.349),
        BACK_LEFT(Rotation2d.fromDegrees(57.079), 8.349);

        private final Rotation2d targetPitch;
        private final double targetShootingVelocityMetersPerSecond;

        FixedShootingPosition(Rotation2d targetPitch, double targetShootingVelocityMetersPerSecond) {
            this.targetPitch = targetPitch;
            this.targetShootingVelocityMetersPerSecond = targetShootingVelocityMetersPerSecond;
        }
    }
}