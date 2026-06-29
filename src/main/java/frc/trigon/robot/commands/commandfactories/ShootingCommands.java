package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.TrenchDetection;
import frc.trigon.robot.misc.matchTracker.MatchTracker;
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
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static FixedShootingPosition TARGET_FIXED_SHOOTING_AT_HUB_STATE = FixedShootingPosition.IN_FRONT_OF_TOWER;
    public static LoggedNetworkBoolean SHOULD_OVERRIDE_GAME_DATA = new LoggedNetworkBoolean("/SmartDashboard/MatchTracker/IsGameDataOverridden", false);
    public static LoggedNetworkBoolean SHOULD_OVERRIDE_SWERVE_AIM = new LoggedNetworkBoolean("/SmartDashboard/IsSwerveAimOverridden", false);

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
                ).until(() -> !RobotContainer.HOOD.atTargetAngle() || !RobotContainer.SHOOTER.atTargetVelocity()).repeatedly()
        );
    }

    public static Command getShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getLoadForShootingWhenReadyCommand(() -> SHOOTING_CALCULATIONS.getCurrentTargetShootingLocation().isDelivery),
                        getSetTargetShootingLocationCommand(),
                        getAimSwerveCommand(() -> SHOOTING_CALCULATIONS.getTargetShootingState().targetFieldRelativeYaw()),
                        getAimHoodForShootingCommand(),
                        ShooterCommands.getAimForShootingCommand(),
                        getIntakeSequenceWhileShootingCommand()
                )
        ).beforeStarting(
                () -> CommandConstants.setSwerveSpeedMultiplier(CommandConstants.SHOOTING_SWERVE_SPEED_MULTIPLIER)
        ).finallyDo(
                () -> CommandConstants.setSwerveSpeedMultiplier(CommandConstants.DEFAULT_SWERVE_SPEED_MULTIPLIER)
        );
    }

    public static Command getFixedShootingAtHubCommand() {
        return new ParallelCommandGroup(
                getLoadForFixedShootingAtHubWhenReadyCommand(),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/isReadyForFixedShootingAtHub", isReadyForFixedShootingAtHub())),
                getAimHoodForFixedShootingCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetPitch),
                ShooterCommands.getSetTargetVelocityCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetShootingVelocityMetersPerSecond),
                GeneralCommands.getContinuousConditionalCommand(
                        getAimSwerveWithOverrideCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetFieldRelativeYaw.get()),
                        SwerveCommands.getLockSwerveCommand(),
                        RobotContainer.SWERVE::isMoving
                ),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", TARGET_FIXED_SHOOTING_AT_HUB_STATE.name())),
                getIntakeSequenceWhileShootingCommand()
        );
    }

    public static Command getFixedDeliveryShootingCommand() {
        return new ParallelCommandGroup(
                getLoadForFixedDeliveryWhenReadyCommand(),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/isReadyForFixedDelivery", isReadyForFixedDelivery())),
                getAimHoodForFixedShootingCommand(() -> HoodConstants.FIXED_DELIVERY_SHOOTING_HOOD_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(() -> ShooterConstants.FIXED_DELIVERY_SHOOTING_SHOOTER_VELOCITY_METERS_PER_SECOND),
                getAimSwerveWithOverrideCommand(SwerveConstants.FIXED_DELIVERY_TARGET_FIELD_RELATIVE_YAW::get),
                getIntakeSequenceWhileShootingCommand()
        );
    }

    public static Command getSetFixedShootingStateCommand(FixedShootingPosition targetState) {
        return new InstantCommand(() -> {
            setTargetFixedShootingAtHubState(targetState);
            Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", targetState.name());
        }).ignoringDisable(true);
    }

    public static Command getSetClosestTrenchCommand() {
        return new InstantCommand(() -> {
            final FixedShootingPosition target = getClosestTrench();
            setTargetFixedShootingAtHubState(target);
            Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", target.name());
        }).ignoringDisable(true);
    }

    public static Command getSetClosestSideOfTowerCommand() {
        return new InstantCommand(() -> {
            final FixedShootingPosition target = getClosestSideOfTower();
            setTargetFixedShootingAtHubState(target);
            Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", target.name());
        }).ignoringDisable(true);
    }

    public static Command getResetPoseToFixedShootingLocationCommand() {
        return new InstantCommand(
                () -> RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(new Pose2d(TARGET_FIXED_SHOOTING_AT_HUB_STATE.resetPose.get(), RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation()))
        ).ignoringDisable(true);
    }

    public static Command getPrepareForShootingCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        getUpdateShootingCalculationsCommand(),
                        getAimHoodForShootingCommand(),
                        ShooterCommands.getAimForShootingCommand()
                )
        );
    }

    public static Command getPrepareForFixedShootingCommand() {
        return new ParallelCommandGroup(
                HoodCommands.getSetTargetAngleCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetPitch),
                ShooterCommands.getSetTargetVelocityCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetShootingVelocityMetersPerSecond),
                getAimSwerveWithOverrideCommand(() -> TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetFieldRelativeYaw.get()),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", TARGET_FIXED_SHOOTING_AT_HUB_STATE.name()))
        );
    }

    public static Command getBasicFixedAutonomousShootingCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> 0.0,
                        () -> 0.0,
                        () -> getAutonomousBasicPosition().targetFieldRelativeYaw
                ),
                GeneralCommands.getContinuousConditionalCommand(
                        HoodCommands.getRestCommand(),
                        HoodCommands.getSetTargetAngleCommand(() -> getAutonomousBasicPosition().targetPitch),
                        TrenchDetection::isHoodInTrenchZone
                ),
                ShooterCommands.getSetTargetVelocityCommand(() -> getAutonomousBasicPosition().targetShootingVelocityMetersPerSecond),
                GeneralCommands.getContinuousConditionalCommand(
                        new ParallelCommandGroup(
                                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING),
                                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING)
                        ),
                        new ParallelCommandGroup(
                                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.REST),
                                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.REST)
                        ),
                        () -> RobotContainer.HOOD.atAngle(getAutonomousBasicPosition().targetPitch) &&
                                RobotContainer.SHOOTER.atVelocity(getAutonomousBasicPosition().targetShootingVelocityMetersPerSecond)
                )
        );
    }

    public static Command getDoubleSwipeFixedAutonomousShootingCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> 0.0,
                        () -> 0.0,
                        () -> getAutonomousDoubleSwipePosition().targetFieldRelativeYaw
                ),
                GeneralCommands.getContinuousConditionalCommand(
                        HoodCommands.getRestCommand(),
                        HoodCommands.getSetTargetAngleCommand(() -> getAutonomousDoubleSwipePosition().targetPitch),
                        TrenchDetection::isHoodInTrenchZone
                ),
                ShooterCommands.getSetTargetVelocityCommand(() -> getAutonomousDoubleSwipePosition().targetShootingVelocityMetersPerSecond),
                GeneralCommands.getContinuousConditionalCommand(
                        new ParallelCommandGroup(
                                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING),
                                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING)
                        ),
                        new ParallelCommandGroup(
                                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.REST),
                                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.REST)
                        ),
                        () -> RobotContainer.HOOD.atAngle(getAutonomousDoubleSwipePosition().targetPitch) &&
                                RobotContainer.SHOOTER.atVelocity(getAutonomousDoubleSwipePosition().targetShootingVelocityMetersPerSecond)
                )
        );
    }

    public static Command getPrepareForDoubleSwipeFixedAutonomousShootingCommand() {
        return new ParallelCommandGroup(
                GeneralCommands.getContinuousConditionalCommand(
                        HoodCommands.getRestCommand(),
                        HoodCommands.getSetTargetAngleCommand(() -> getAutonomousDoubleSwipePosition().targetPitch),
                        TrenchDetection::isHoodInTrenchZone
                ),
                ShooterCommands.getSetTargetVelocityCommand(() -> getAutonomousDoubleSwipePosition().targetShootingVelocityMetersPerSecond),
                new RunCommand(() -> Logger.recordOutput("ShootingCalculations/FixedShootingAtHubState", getAutonomousDoubleSwipePosition()))
        );
    }

    private static FixedShootingPosition getAutonomousDoubleSwipePosition() {
        return AutonomousCommands.IS_AUTO_LEFT_SIDE
                ? FixedShootingPosition.AUTONOMOUS_DOUBLE_SWIPE_LEFT
                : FixedShootingPosition.AUTONOMOUS_DOUBLE_SWIPE_RIGHT;
    }

    private static FixedShootingPosition getAutonomousBasicPosition() {
        return FixedShootingPosition.AUTONOMOUS_BASIC_LEFT;
//        return AutonomousCommands.IS_AUTO_LEFT_SIDE
//                ? FixedShootingPosition.AUTONOMOUS_BASIC_LEFT
//                : FixedShootingPosition.AUTONOMOUS_BASIC_RIGHT;
    }

    public static RepeatCommand getIntakeSequenceWhileShootingCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).until(OperatorConstants.CLOSE_INTAKE_WHILE_SHOOTING_TRIGGER),
                FuelIntakeCommands.getCloseIntakeWhileShootingCommand().until(OperatorConstants.INTAKE_WHILE_SHOOTING_TRIGGER)
        ).repeatedly();
    }

    public static Command getEnableOverrideSwerveAimCommand() {
        return new InstantCommand(ShootingCommands::enableOverrideSwerveAim).ignoringDisable(true);
    }

    public static Command getDisableOverrideSwerveAimCommand() {
        return new InstantCommand(ShootingCommands::disableOverrideSwerveAim).ignoringDisable(true);
    }

    private static Command getLoadForFixedShootingAtHubWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(() -> false).until(() -> !isReadyForFixedShootingAtHub()),
                ShootingCommands::isReadyForFixedShootingAtHub
        ).repeatedly();
    }

    private static Command getLoadForFixedDeliveryWhenReadyCommand() {
        return GeneralCommands.runWhen(
                getLoadForShootingCommand(() -> true).until(() -> !isReadyForFixedDelivery()),
                ShootingCommands::isReadyForFixedDelivery
        ).repeatedly();
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

    private static Command getAimHoodForFixedShootingCommand(Supplier<Rotation2d> targetAngleSupplier) {
        return GeneralCommands.getContinuousConditionalCommand(
                HoodCommands.getRestCommand(),
                HoodCommands.getSetTargetAngleCommand(targetAngleSupplier),
                OperatorConstants.LOWER_HOOD_TRIGGER
        );
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(ShootingCommands::updateShootingCalculations);
    }

    private static Command getAimSwerveWithOverrideCommand(Supplier<Rotation2d> rotationSupplier) {
        return GeneralCommands.getContinuousConditionalCommand(
                GeneralCommands.getFieldRelativeDriveCommand(),
                getAimSwerveCommand(rotationSupplier),
                () -> SHOULD_OVERRIDE_SWERVE_AIM.getAsBoolean()
        );
    }

    private static Command getAimSwerveCommand(Supplier<Rotation2d> rotationSupplier) {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> new FlippableRotation2d(rotationSupplier.get(), false)
        );
    }

    private static Command getAimHoodForShootingCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HoodCommands.getRestCommand(),
                HoodCommands.getAimForShootingCommand(),
                TrenchDetection::isHoodInTrenchZone
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
        final boolean isAngleReady = isSwerveAtAngle(SwerveConstants.FIXED_DELIVERY_TARGET_FIELD_RELATIVE_YAW);

        return isPitchReady && isVelocityReady && isAngleReady;
    }

    private static boolean isReadyForFixedShootingAtHub() {
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetPitch);
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetShootingVelocityMetersPerSecond);
        final boolean isAngleReady = isSwerveAtAngle(TARGET_FIXED_SHOOTING_AT_HUB_STATE.targetFieldRelativeYaw);

        return isPitchReady && isVelocityReady && isAngleReady;
    }

    private static boolean isSwerveAtAngle(FlippableRotation2d targetFieldRelativeYaw) {
        if (SHOULD_OVERRIDE_SWERVE_AIM.getAsBoolean())
            return true;

        return RobotContainer.SWERVE.atAngle(targetFieldRelativeYaw);
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

    private static FixedShootingPosition getClosestTrench() {
        return FieldConstants.isRight() ? FixedShootingPosition.RIGHT_TRENCH : FixedShootingPosition.LEFT_TRENCH;
    }

    private static FixedShootingPosition getClosestSideOfTower() {
        final Translation2d robotPosition = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d rightOfTowerPosition = FieldConstants.RIGHT_OF_TOWER_POSITION.get();
        final Translation2d leftOfTowerPosition = FieldConstants.LEFT_OF_TOWER_POSITION.get();

        final double distanceToRight = robotPosition.getDistance(rightOfTowerPosition);
        final double distanceToLeft = robotPosition.getDistance(leftOfTowerPosition);

        return distanceToRight <= distanceToLeft ? FixedShootingPosition.RIGHT_OF_TOWER : FixedShootingPosition.LEFT_OF_TOWER;
    }

    private static void setTargetFixedShootingAtHubState(FixedShootingPosition targetFixedShootingAtHubState) {
        TARGET_FIXED_SHOOTING_AT_HUB_STATE = targetFixedShootingAtHubState;
    }

    private static boolean isReadyForShooting(BooleanSupplier isDelivery) {
        if (!SHOOTING_CALCULATIONS.isReadyToShoot())
            return false;

        if (isDelivery.getAsBoolean())
            return !isDeliveryHittingHub();

        return MatchTracker.isHubActive();
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

    public static void enableOverrideGameData() {
        SHOULD_OVERRIDE_GAME_DATA.set(true);
    }

    public static void disableOverrideGameData() {
        SHOULD_OVERRIDE_GAME_DATA.set(false);
    }

    public static void enableOverrideSwerveAim() {
        SHOULD_OVERRIDE_SWERVE_AIM.set(true);
    }

    public static void disableOverrideSwerveAim() {
        SHOULD_OVERRIDE_SWERVE_AIM.set(false);
    }

    public enum FixedShootingPosition { // TODO: Get all values from shooting calculations IRL
        IN_FRONT_OF_TOWER(Rotation2d.fromDegrees(25), 7.6, Rotation2d.fromDegrees(-177.967), new Translation2d(2.422, 3.948)),
        BETWEEN_TOWER_AND_HUB(Rotation2d.fromDegrees(23), 7.1, Rotation2d.fromDegrees(177.9), new Translation2d(1.777, 3.948)),
        RIGHT_TRENCH(Rotation2d.fromDegrees(28), 8.55, Rotation2d.fromDegrees(-100.108), new Translation2d(3.939, 0.776326)),
        LEFT_TRENCH(Rotation2d.fromDegrees(28), 8.55, Rotation2d.fromDegrees(100.108), new Translation2d(3.939, 7.293)),
        RIGHT_OF_TOWER(Rotation2d.fromDegrees(28), 9.1, Rotation2d.fromDegrees(-157.736), new Translation2d(0.716, 4.827)),
        LEFT_OF_TOWER(Rotation2d.fromDegrees(28), 8.95, Rotation2d.fromDegrees(166.564), new Translation2d(0.949, 2.531)),
        AUTONOMOUS_DOUBLE_SWIPE_LEFT(Rotation2d.fromDegrees(28), 8.55, Rotation2d.fromDegrees(120.390), new Translation2d(2.853, 6.768)),
        AUTONOMOUS_DOUBLE_SWIPE_RIGHT(Rotation2d.fromDegrees(28), 8.55, Rotation2d.fromDegrees(-120.390), new Translation2d(2.853, FieldConstants.FIELD_WIDTH_METERS - 6.768)),
        AUTONOMOUS_BASIC_LEFT(Rotation2d.fromDegrees(23), 7.1, Rotation2d.fromDegrees(180), new Translation2d(1.777, 3.948)),
        AUTONOMOUS_BASIC_RIGHT(Rotation2d.fromDegrees(23), 7.1, Rotation2d.fromDegrees(180), new Translation2d(1.777, 3.948));

        private final Rotation2d targetPitch;
        private final double targetShootingVelocityMetersPerSecond;
        private final FlippableRotation2d targetFieldRelativeYaw;
        private final FlippableTranslation2d resetPose;

        FixedShootingPosition(Rotation2d targetPitch, double targetShootingVelocityMetersPerSecond, Rotation2d targetFieldRelativeYaw, Translation2d position) {
            this.targetPitch = targetPitch;
            this.targetShootingVelocityMetersPerSecond = targetShootingVelocityMetersPerSecond;
            this.targetFieldRelativeYaw = new FlippableRotation2d(targetFieldRelativeYaw, true);
            this.resetPose = new FlippableTranslation2d(position, true);
        }
    }
}