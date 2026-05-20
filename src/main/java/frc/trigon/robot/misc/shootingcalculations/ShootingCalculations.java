package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
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
    public Translation3d calculateCurrentFuelExitPose() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Rotation2d shooterPitch = RobotContainer.HOOD.getCurrentAngle();
        return calculateFieldRelativeFuelExitPose(robotPose, shooterPitch);
    }

    public Translation3d calculateFieldRelativeFuelExitPose(Pose2d robotPose, Rotation2d pitch) {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -pitch.getRadians(), 0)
        );

        final Pose3d shooterOriginPose = new Pose3d(robotPose).transformBy(RobotContainer.SHOOTER.getComponenetPose());
        final Pose3d pitchedShooterPose = shooterOriginPose.transformBy(pitchTransform);

        return pitchedShooterPose.transformBy(ShootingCalculationsConstants.SHOOTER_TO_FUEL_EXIT).getTranslation();
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