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

    // Default to the Hub
    private TargetLocation currentTargetLocation = TargetLocation.HUB;

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null) INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void setTargetShootingLocation(TargetLocation newTarget) {
        this.currentTargetLocation = newTarget;
    }

    public TargetLocation getCurrentTargetShootingLocation() {
        return currentTargetLocation;
    }

    public void updateCalculations() {
        targetShootingState = calculateTargetShootingState();

        Logger.recordOutput("Shooting/TargetShootingYawDegrees", targetShootingState.targetFieldRelativeYaw().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingPitchDegrees", targetShootingState.targetPitch().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingVelocityMPS", targetShootingState.targetShootingVelocityMetersPerSecond());
        Logger.recordOutput("Shooting/TargetMode", currentTargetLocation.name());
        Logger.recordOutput("Shooting/IsReadyToShoot", isReadyToShoot());
    }

    public ShootingState getTargetShootingState() {
        return targetShootingState;
    }

    /**
     * @return True if the chassis, hood pitch, and shooter wheels are all at their PID setpoints.
     */
    @AutoLogOutput(key = "Shooting/isReadyToShoot")
    public boolean isReadyToShoot() {
        final boolean isYawReady = RobotContainer.SWERVE.atAngle(new FlippableRotation2d(targetShootingState.targetFieldRelativeYaw(), false));
        final boolean isPitchReady = RobotContainer.HOOD.atAngle(targetShootingState.targetPitch());
        final boolean isVelocityReady = RobotContainer.SHOOTER.atTargetVelocity();

        return isYawReady && isPitchReady && isVelocityReady;
    }

    @AutoLogOutput(key = "Shooting/CurrentFuelExitPosition")
    public Translation3d calculateCurrentFuelExitPose(int columnIndex) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Rotation2d shooterPitch = RobotContainer.HOOD.getCurrentAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, shooterPitch, columnIndex);
    }

    public Translation3d calculateFieldRelativeFuelExitPose(Pose2d robotPose, Rotation2d pitch, int columnIndex) {
        // 1. Calculate how far left or right the ball is from the center of the drum
        double colOffset = (columnIndex - (SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY - 1) / 2.0) * SimulatedGamePieceConstants.INDEXER_COL_SPACING_METERS;

        // 2. Apply that lateral shift
        Transform3d laneSpecificExitTransform = new Transform3d(
                new Translation3d(0, colOffset, 0),
                new Rotation3d()
        );

        // 3. Apply the pitch rotation to the exit pose
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -pitch.getRadians(), 0)
        );

        final Pose3d baseExitPose = new Pose3d(robotPose).transformBy(ShooterConstants.FUEL_EXIT_SHOOTER_POSE);
        final Pose3d pitchedExitPose = baseExitPose.transformBy(pitchTransform);

        // 4. Combine the pitched pose with the lane offset
        return pitchedExitPose.transformBy(laneSpecificExitTransform).getTranslation();
    }

    private ShootingState calculateTargetShootingState() {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.POSE_PREDICTION_TIME_SECONDS);
        final ChassisSpeeds fieldRelativeChassisSpeeds = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds();
        return calculateTargetShootingState(predictedRobotPose, fieldRelativeChassisSpeeds);
    }

    public ShootingState calculateTargetShootingState(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {
        final Translation2d targetPhysicalPosition = currentTargetLocation.position.get();
        final Translation2d currentRobotPosition = robotPose.getTranslation();
        final Translation2d robotVelocity = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);

        Translation2d virtualTarget = targetPhysicalPosition;
        double distanceToVirtualTarget = currentRobotPosition.getDistance(virtualTarget);

        ShotParameters parameters = ShootingMap.getInterpolatedParameters(distanceToVirtualTarget, currentTargetLocation.isDelivery);

        for (int i = 0; i < ShootingCalculationsConstants.VIRTUAL_HUB_CALCULATION_ITERATIONS; i++) {
            virtualTarget = targetPhysicalPosition.minus(robotVelocity.times(parameters.timeOfFlight()));
            distanceToVirtualTarget = currentRobotPosition.getDistance(virtualTarget);
            parameters = ShootingMap.getInterpolatedParameters(distanceToVirtualTarget, currentTargetLocation.isDelivery);
        }

        final Rotation2d targetYaw = virtualTarget.minus(currentRobotPosition).getAngle();

        Logger.recordOutput("Shooting/DistanceToVirtualTarget", distanceToVirtualTarget);
        Logger.recordOutput("Shooting/InterpolatedTimeOfFlight", parameters.timeOfFlight());

        return new ShootingState(
                targetYaw,
                parameters.pitch(),
                parameters.velocity()
        );
    }

    public enum TargetLocation {
        HUB(FieldConstants.HUB_POSITION, false),
        RIGHT_DELIVERY_LOCATION(FieldConstants.RIGHT_DELIVERY_POSITION, true),
        LEFT_DELIVERY_LOCATION(FieldConstants.LEFT_DELIVERY_POSITION, true);

        public final FlippableTranslation2d position;
        public final boolean isDelivery;

        TargetLocation(FlippableTranslation2d position, boolean isDelivery) {
            this.position = position;
            this.isDelivery = isDelivery;
        }
    }
}