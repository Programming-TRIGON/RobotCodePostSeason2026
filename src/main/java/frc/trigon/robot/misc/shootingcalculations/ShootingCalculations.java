package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    private ShootingState targetShootingState = ShootingState.empty();

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void updateCalculations() {
        targetShootingState = calculateTargetShootingState();

        Logger.recordOutput("Shooting/TargetShootingYawDegrees", targetShootingState.targetFieldRelativeYaw().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingPitchDegrees", targetShootingState.targetPitch().getDegrees());
        Logger.recordOutput("Shooting/TargetShootingVelocityMPS", targetShootingState.targetShootingVelocityMetersPerSecond());
    }

    public ShootingState getTargetShootingState() {
        return targetShootingState;
    }

    @AutoLogOutput(key = "Shooting/CurrentFuelExitPosition")
    public Translation3d calculateCurrentFuelExitPose(int columnIndex) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Rotation2d shooterPitch = RobotContainer.HOOD.getCurrentAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, shooterPitch, columnIndex);
    }

    public Translation3d calculateFieldRelativeFuelExitPose(Pose2d robotPose, Rotation2d pitch, int columnIndex) {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -pitch.getRadians(), 0)
        );

        // NEW: Calculate how far left or right the ball is from the center of the drum
        // Columns 0, 1, 2, 3 become Offsets: -1.5, -0.5, 0.5, 1.5 multiplied by the spacing
        Transform3d laneSpecificExitTransform = getLaneSpecificExitTransform(columnIndex);

        final Pose3d shooterOriginPose = new Pose3d(robotPose).transformBy(ShooterConstants.FUEL_EXIT_SHOOTER_POSE);
        final Pose3d pitchedShooterPose = shooterOriginPose.transformBy(pitchTransform);

        // Use the new lane-specific transform to pinpoint the 3D exit location
        return pitchedShooterPose.transformBy(laneSpecificExitTransform).getTranslation();
    }

    private static Transform3d getLaneSpecificExitTransform(int columnIndex) {
        double colOffset = (columnIndex - (SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY - 1) / 2.0) * SimulatedGamePieceConstants.INDEXER_COL_SPACING_METERS;

        // Apply that lateral shift to the base shooter exit position
        Translation3d laneSpecificExitTranslation = new Translation3d(
                ShootingCalculationsConstants.SHOOTER_TO_FUEL_EXIT.getX(),
                ShootingCalculationsConstants.SHOOTER_TO_FUEL_EXIT.getY() + colOffset,
                ShootingCalculationsConstants.SHOOTER_TO_FUEL_EXIT.getZ()
        );

        return new Transform3d(
                laneSpecificExitTranslation,
                ShootingCalculationsConstants.SHOOTER_TO_FUEL_EXIT.getRotation()
        );
    }

    private ShootingState calculateTargetShootingState() {
        final Pose2d predictedRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(ShootingCalculationsConstants.POSE_PREDICTION_TIME_SECONDS);
        final ChassisSpeeds fieldRelativeChassisSpeeds = RobotContainer.SWERVE.getFieldRelativeChassisSpeeds();
        return calculateTargetShootingState(predictedRobotPose, fieldRelativeChassisSpeeds);
    }

    public ShootingState calculateTargetShootingState(Pose2d robotPose, ChassisSpeeds fieldRelativeChassisSpeeds) {
        final Translation2d physicalHubPosition = FieldConstants.HUB_POSITION.get();
        final Translation2d currentRobotPosition = robotPose.getTranslation();
        final Translation2d robotVelocity = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);

        Translation2d virtualHub = physicalHubPosition;
        double distanceToVirtualHub = currentRobotPosition.getDistance(virtualHub);

        ShotParameters parameters = ShootingMap.getInterpolatedParameters(distanceToVirtualHub);

        for (int i = 0; i < ShootingCalculationsConstants.VIRTUAL_HUB_CALCULATION_ITERATIONS; i++) {
            virtualHub = physicalHubPosition.minus(robotVelocity.times(parameters.timeOfFlight()));
            distanceToVirtualHub = currentRobotPosition.getDistance(virtualHub);
            parameters = ShootingMap.getInterpolatedParameters(distanceToVirtualHub);
        }

        final Rotation2d targetYaw = virtualHub.minus(currentRobotPosition).getAngle();

        Logger.recordOutput("Shooting/DistanceToVirtualHub", distanceToVirtualHub);
        Logger.recordOutput("Shooting/InterpolatedTimeOfFlight", parameters.timeOfFlight());

        return new ShootingState(
                targetYaw,
                parameters.pitch(),
                parameters.velocity()
        );
    }
}