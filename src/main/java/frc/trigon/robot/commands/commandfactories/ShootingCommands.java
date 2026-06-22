package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> {
            setTargetFixedShootingAtHubState(targetState);
            Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", targetState.name());
        });
    }

    public static Command getPrepareForShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getAimForShootingCommand()
                )
        );
    }

    public static RepeatCommand getIntakeSequenceWhileShootingCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).until(OperatorConstants.CLOSE_INTAKE_WHILE_SHOOTING_TRIGGER),
                FuelIntakeCommands.getCloseIntakeWhileShootingCommand().until(OperatorConstants.INTAKE_WHILE_SHOOTING_TRIGGER)
        ).repeatedly();
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
                getLoadForShootingCommand(isDelivery).until(() -> !isReadyForShooting(isDelivery)),
                () -> isReadyForShooting(isDelivery)
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

    private static Command getAimSwerveCommand(Supplier<Rotation2d> rotationSupplier) {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> new FlippableRotation2d(rotationSupplier.get(), false)
        );
    }

    private static Command getAimForShootingCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getAimForShootingCommand(),
                ShooterCommands.getAimForShootingCommand()
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

    public enum FixedShootingPosition { // TODO: Get all values from shooting calculations IRL
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

    private static boolean isReadyForShooting(BooleanSupplier isDelivery) {
        return SHOOTING_CALCULATIONS.isReadyToShoot() && (!isDelivery.getAsBoolean() || !isDeliveryHittingHub());
    }

    private static boolean isDeliveryHittingHub() {
        final ShootingCalculations.TargetShootingLocation currentTarget = SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation();

        if (!currentTarget.isDelivery)
            return false;

        return isDeliveryHittingHub(currentTarget);
    }

    private static boolean isDeliveryHittingHub(ShootingCalculations.TargetShootingLocation targetLocation) {
        final Translation2d robotPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d deliveryPosition = getDeliveryPosition(targetLocation);

        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();
        final Translation2d oppositesHubPosition = getFlippedHubPosition();

        final boolean isHittingRegularHub = doesDeliveryHitHub(robotPosition, deliveryPosition, hubPosition);
        final boolean isHittingOppositeHub = doesDeliveryHitHub(robotPosition, deliveryPosition, oppositesHubPosition);
        final boolean isHittingHub = isHittingRegularHub || isHittingOppositeHub;

        Logger.recordOutput("Shooting/Delivery/HittingHub/IsHittingHub", isHittingHub);
        Logger.recordOutput("Shooting/Delivery/HittingHub/TargetLocation", targetLocation.name());

        return isHittingHub;
    }

    private static Translation2d getFlippedHubPosition() {
        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();

        return new Translation2d(
                FieldConstants.FIELD_LENGTH_METERS - hubPosition.getX(),
                FieldConstants.FIELD_WIDTH_METERS - hubPosition.getY()
        );
    }

    private static boolean doesDeliveryHitHub(Translation2d robotPosition, Translation2d deliveryPosition, Translation2d hubPosition) {
        final double minimumX = hubPosition.getX() - FieldConstants.HALF_SIZE_OF_HUB_CENTER_X;
        final double maximumX = hubPosition.getX() + FieldConstants.HALF_SIZE_OF_HUB_CENTER_X;
        final double minimumY = hubPosition.getY() - FieldConstants.HALF_SIZE_OF_HUB_CENTER_X - FieldConstants.EXTRA_HUB_WIDTH;
        final double maximumY = hubPosition.getY() + FieldConstants.HALF_SIZE_OF_HUB_CENTER_X + FieldConstants.EXTRA_HUB_WIDTH;

        return doesDeliveryShootingLineIntersectHub(
                robotPosition,
                deliveryPosition,
                minimumX,
                maximumX,
                minimumY,
                maximumY
        );
    }

    private static Translation2d getDeliveryPosition(ShootingCalculations.TargetShootingLocation targetLocation) {
        if (targetLocation == ShootingCalculations.TargetShootingLocation.RIGHT_DELIVERY_LOCATION)
            return FieldConstants.RIGHT_DELIVERY_POSITION.get();

        return FieldConstants.LEFT_DELIVERY_POSITION.get();
    }

    private static boolean doesDeliveryShootingLineIntersectHub(
            Translation2d deliveryPathStart,
            Translation2d deliveryPathEnd,
            double hubRectangleMinimumX,
            double hubRectangleMaximumX,
            double hubRectangleMinimumY,
            double hubRectangleMaximumY
    ) {
        final Translation2d bottomLeftHubCorner = new Translation2d(hubRectangleMinimumX, hubRectangleMinimumY);
        final Translation2d bottomRightHubCorner = new Translation2d(hubRectangleMaximumX, hubRectangleMinimumY);
        final Translation2d topLeftHubCorner = new Translation2d(hubRectangleMinimumX, hubRectangleMaximumY);
        final Translation2d topRightHubCorner = new Translation2d(hubRectangleMaximumX, hubRectangleMaximumY);

        return doLinesIntersect(deliveryPathStart, deliveryPathEnd, bottomLeftHubCorner, bottomRightHubCorner) ||
                doLinesIntersect(deliveryPathStart, deliveryPathEnd, bottomRightHubCorner, topRightHubCorner) ||
                doLinesIntersect(deliveryPathStart, deliveryPathEnd, topRightHubCorner, topLeftHubCorner) ||
                doLinesIntersect(deliveryPathStart, deliveryPathEnd, topLeftHubCorner, bottomLeftHubCorner);
    }

    private static boolean doLinesIntersect(
            Translation2d deliveryPathStartPoint,
            Translation2d deliveryPathEndPoint,
            Translation2d hubSideStartPoint,
            Translation2d hubSideEndPoint
    ) {
        final double firstDeltaX = deliveryPathEndPoint.getX() - deliveryPathStartPoint.getX();
        final double firstDeltaY = deliveryPathEndPoint.getY() - deliveryPathStartPoint.getY();
        final double secondDeltaX = hubSideEndPoint.getX() - hubSideStartPoint.getX();
        final double secondDeltaY = hubSideEndPoint.getY() - hubSideStartPoint.getY();

        final double denominator = (-secondDeltaX * firstDeltaY + firstDeltaX * secondDeltaY);

        if (denominator == 0)
            return false;

        final double firstIntersection =
                (-firstDeltaY * (deliveryPathStartPoint.getX() - hubSideStartPoint.getX()) + firstDeltaX * (deliveryPathStartPoint.getY() - hubSideStartPoint.getY())) / denominator;
        final double secondIntersection =
                (secondDeltaX * (deliveryPathStartPoint.getY() - hubSideStartPoint.getY()) - secondDeltaY * (deliveryPathStartPoint.getX() - hubSideStartPoint.getX())) / denominator;

        return firstIntersection >= 0 && firstIntersection <= 1 && secondIntersection >= 0 && secondIntersection <= 1;
    }
}