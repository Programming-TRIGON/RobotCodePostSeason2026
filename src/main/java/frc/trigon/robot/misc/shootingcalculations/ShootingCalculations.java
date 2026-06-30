package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    private ShootingState targetShootingState = ShootingState.empty();
    private TargetShootingLocation currentTargetShootingLocation = TargetShootingLocation.HUB;

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null) INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void setTargetShootingLocation(TargetShootingLocation newTarget) {
        this.currentTargetShootingLocation = newTarget;
    }

    public TargetShootingLocation getCurrentTargetShootingLocation() {
        return currentTargetShootingLocation;
    }

    public void updateCalculations() {
        targetShootingState = calculateTargetShootingState();

        Logger.recordOutput("ShootingCalculations/TargetShootingYawDegrees", targetShootingState.targetFieldRelativeYaw().getDegrees());
        Logger.recordOutput("ShootingCalculations/TargetShootingPitchDegrees", targetShootingState.targetPitch().getDegrees());
        Logger.recordOutput("ShootingCalculations/TargetShootingVelocityMPS", targetShootingState.targetShootingVelocityMetersPerSecond());
        Logger.recordOutput("ShootingCalculations/TargetMode", currentTargetShootingLocation.name());
        Logger.recordOutput("ShootingCalculations/Conditions/SwerveAtTargetAngle", RobotContainer.SWERVE.atAngle(new FlippableRotation2d(targetShootingState.targetFieldRelativeYaw(), false)));

        logRangeToAllTargets();
    }

    public ShootingState getTargetShootingState() {
        return targetShootingState;
    }

    /**
     * @return True if the chassis, hood pitch, and shooter wheels are all at their PID setpoints.
     */
    @AutoLogOutput(key = "ShootingCalculations/isReadyToShoot")
    public boolean isReadyToShoot() {
        final boolean isYawReady = RobotContainer.SWERVE.atAngle(new FlippableRotation2d(targetShootingState.targetFieldRelativeYaw(), false));
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(targetShootingState.targetPitch());
        final boolean isVelocityReady = RobotContainer.SHOOTER.atVelocity(targetShootingState.targetShootingVelocityMetersPerSecond());

        return isYawReady && isPitchReady && isVelocityReady;
    }

    @AutoLogOutput(key = "ShootingCalculations/CurrentFuelExitPosition")
    public Translation3d calculateCurrentFuelExitPose(int columnIndex) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Rotation2d shooterPitch = RobotContainer.HOOD.getCurrentAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, shooterPitch, columnIndex);
    }

    public Translation3d calculateFieldRelativeFuelExitPose(Pose2d robotPose, Rotation2d pitch, int columnIndex) {
        double colOffset = (columnIndex - (SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY - 1) / 2.0) * SimulatedGamePieceConstants.INDEXER_COL_SPACING_METERS;

        Transform3d laneSpecificExitTransform = new Transform3d(
                new Translation3d(0, colOffset, 0),
                new Rotation3d()
        );

        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -pitch.getRadians(), 0)
        );

        final Pose3d baseExitPose = new Pose3d(robotPose).transformBy(ShooterConstants.FUEL_EXIT_SHOOTER_POSE);
        final Pose3d pitchedExitPose = baseExitPose.transformBy(pitchTransform);

        return pitchedExitPose.transformBy(laneSpecificExitTransform).getTranslation();
    }

    private void logRangeToAllTargets() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d shooterExitXY = new Pose3d(robotPose)
                .transformBy(ShooterConstants.FUEL_EXIT_SHOOTER_POSE)
                .getTranslation().toTranslation2d();

        final double distanceToHub = shooterExitXY.getDistance(FieldConstants.HUB_POSITION.get());
        final double distanceToRightDelivery = shooterExitXY.getDistance(FieldConstants.RIGHT_DELIVERY_POSITION.get());
        final double distanceToLeftDelivery = shooterExitXY.getDistance(FieldConstants.LEFT_DELIVERY_POSITION.get());
        final boolean rightIsCloser = distanceToRightDelivery <= distanceToLeftDelivery;
        final double distanceToClosestDelivery = rightIsCloser ? distanceToRightDelivery : distanceToLeftDelivery;

        Logger.recordOutput("ShootingCalculations/Distance/HubMeters", distanceToHub);
        Logger.recordOutput("ShootingCalculations/Distance/RightDeliveryMeters", distanceToRightDelivery);
        Logger.recordOutput("ShootingCalculations/Distance/LeftDeliveryMeters", distanceToLeftDelivery);
        Logger.recordOutput("ShootingCalculations/Distance/ClosestDeliveryMeters", distanceToClosestDelivery);
        Logger.recordOutput("ShootingCalculations/Distance/ClosestDeliveryName", rightIsCloser ? "RIGHT" : "LEFT");
    }

    private ShootingState calculateTargetShootingState() {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.POSE_PREDICTION_TIME_SECONDS);
        final ChassisSpeeds fieldRelativeChassisSpeeds = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds();
        return calculateTargetShootingState(predictedRobotPose, fieldRelativeChassisSpeeds);
    }

    public ShootingState calculateTargetShootingState(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {
        final Translation2d targetPhysicalPosition = currentTargetShootingLocation.position.get();
        final Translation2d robotVelocity = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);

        // Uses exact shooter exit for both distance interpolation and swerve aim angle.
        final Pose3d baseExitPose = new Pose3d(robotPose).transformBy(ShooterConstants.FUEL_EXIT_SHOOTER_POSE);
        final Translation2d shooterExitFieldPosition = baseExitPose.getTranslation().toTranslation2d();

        // Iteratively converge on a self-consistent (virtualTarget, parameters) pair.
        // Shifting to the virtual target changes the shot distance, which changes the
        // interpolated velocity/pitch/ToF, which changes the virtual target again.
        // Each iteration tightens that coupling; VIRTUAL_TARGET_ITERATIONS = 5 is enough
        // to converge within sub-millimeter error for any realistic robot speed.
        Translation2d virtualTarget = targetPhysicalPosition;
        ShotParameters parameters = ShootingMap.getInterpolatedParameters(
                shooterExitFieldPosition.getDistance(virtualTarget), currentTargetShootingLocation.isDelivery);

        for (int i = 0; i < ShootingCalculationsConstants.VIRTUAL_TARGET_ITERATIONS; i++) {
            virtualTarget = targetPhysicalPosition.minus(robotVelocity.times(parameters.timeOfFlight()));
            parameters = ShootingMap.getInterpolatedParameters(
                    shooterExitFieldPosition.getDistance(virtualTarget), currentTargetShootingLocation.isDelivery);
        }

        final Rotation2d targetYaw = virtualTarget.minus(shooterExitFieldPosition).getAngle().rotateBy(Rotation2d.k180deg);

        Logger.recordOutput("ShootingCalculations/DistanceToVirtualTarget", shooterExitFieldPosition.getDistance(virtualTarget));
        Logger.recordOutput("ShootingCalculations/InterpolatedTimeOfFlight", parameters.timeOfFlight());

        return new ShootingState(
                targetYaw,
                parameters.pitch(),
                parameters.velocity()
        );
    }

    public enum TargetShootingLocation {
        HUB(FieldConstants.HUB_POSITION, false),
        RIGHT_DELIVERY_LOCATION(FieldConstants.RIGHT_DELIVERY_POSITION, true),
        LEFT_DELIVERY_LOCATION(FieldConstants.LEFT_DELIVERY_POSITION, true);

        public final FlippableTranslation2d position;
        public final boolean isDelivery;

        TargetShootingLocation(FlippableTranslation2d position, boolean isDelivery) {
            this.position = position;
            this.isDelivery = isDelivery;
        }
    }
}