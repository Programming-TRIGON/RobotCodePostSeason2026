package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

public class CustomCenterOfRotationDrive implements DriveRestriction {
    private final Translation2d centerOfRotationOffsetMeters;

    public CustomCenterOfRotationDrive(Translation2d centerOfRotationOffsetMeters) {
        this.centerOfRotationOffsetMeters = centerOfRotationOffsetMeters;
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double rotationMagnitude = CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX());
        final double omegaRadiansPerSecond = rotationMagnitude * SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND;

        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final Translation2d offsetInField = centerOfRotationOffsetMeters.rotateBy(robotHeading);

        final Translation2d compensationMetersPerSecond = new Translation2d(
                omegaRadiansPerSecond * offsetInField.getY(),
                omegaRadiansPerSecond * offsetInField.getX()
        );
        final Translation2d compensationMagnitude = compensationMetersPerSecond.div(SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND);

        return targetTranslation.plus(compensationMagnitude);
    }
}
