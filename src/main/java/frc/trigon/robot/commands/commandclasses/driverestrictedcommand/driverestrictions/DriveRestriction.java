package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A restriction applied to the robot's drive by a DriveRestrictedCommand.
 * Each restriction changes either the translation, rotation, or center of rotation.
 * All methods default to passing the value through unchanged.
 */
public interface DriveRestriction {
    /**
     * Applies a restriction to the target translation of the robot.
     *
     * @param targetTranslation the target translation of the robot
     * @return the restricted target translation of the robot
     */
    default Translation2d applyTranslationRestriction(Translation2d targetTranslation) {
        return targetTranslation;
    }

    /**
     * Applies a restriction to the target rotation of the robot.
     *
     * @param targetRotation the target rotation of the robot
     * @return the restricted target rotation of the robot
     */
    default double applyRotationRestriction(double targetRotation) {
        return targetRotation;
    }

    /**
     * Applies a restriction to the target center of rotation of the robot.
     *
     * @param targetCenterOfRotation the target center of rotation
     * @return the robot's restricted target center of rotation
     */
    default Translation2d applyCenterOfRotationRestriction(Translation2d targetCenterOfRotation) {
        return targetCenterOfRotation;
    }

    /**
     * Resets the restricted values so that stale values aren't used.
     */
    default void init() {
    }
}