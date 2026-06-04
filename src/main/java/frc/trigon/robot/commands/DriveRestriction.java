package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;

public interface DriveRestriction {
    /**
     * Applies a restriction to the target translation of the robot
     *
     * @param targetTranslation The target translation of the robot.
     * @return The restricted target translation of the robot.
     */
    default Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        return targetTranslation;
    }

    /**
     * Applies a restriction to the target rotation of the robot.
     *
     * @param targetTheta The target rotation of the robot.
     * @return The restricted target rotation of the robot.
     */
    default double applyRestrictionToTheta(double targetTheta) {
        return targetTheta;
    }
}