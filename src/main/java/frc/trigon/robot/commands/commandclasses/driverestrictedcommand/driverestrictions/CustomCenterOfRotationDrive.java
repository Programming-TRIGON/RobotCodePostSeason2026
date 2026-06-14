package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A restriction that changes the robot's center of rotation.
 * Rotates the robot around the offset point instead of the chassis center.
 * Useful for rotating around an extended mechanism such as an intake.
 */
public class CustomCenterOfRotationDrive implements DriveRestriction {
    private final Translation2d centerOfRotation;

    /**
     * A restriction that changes the robot's center of rotation.
     *
     * @param centerOfRotation the desired robot's center of rotation, in robot-relative coordinates. Positive X is forward, positive Y is left.
     */
    public CustomCenterOfRotationDrive(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    @Override
    public Translation2d getCenterOfRotation() {
        return centerOfRotation;
    }
}
