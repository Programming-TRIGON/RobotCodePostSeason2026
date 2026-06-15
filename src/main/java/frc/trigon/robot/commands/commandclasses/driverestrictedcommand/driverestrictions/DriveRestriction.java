package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.geometry.Translation2d;

public interface DriveRestriction {
    /**
     * Applies a restriction to the target translation of the robot
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
     * Applies a restriction to the target rotation of the robot.
     *
     * @param targetCenterOfRotation the robot's target center of rotation
     * @return the robot's restricted target center of rotation
     */
    default Translation2d applyCenterOfRotationRestriction(Translation2d targetCenterOfRotation) {
        return targetCenterOfRotation;
    }

    /**
     * Is run every time the command is used.
     * Used to reset the values so past values don't affect the command.
     */
    default void init() {}
}