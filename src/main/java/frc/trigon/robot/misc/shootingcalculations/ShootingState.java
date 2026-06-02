package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the final, calculated setpoints for the robot's subsystems.
 * This is the ultimate output of the ShootingCalculations class.
 */
public record ShootingState(
        Rotation2d targetFieldRelativeYaw,
        Rotation2d targetPitch,
        double targetShootingVelocityMetersPerSecond
) {
    public static ShootingState empty() {
        return new ShootingState(
                new Rotation2d(),
                new Rotation2d(),
                0.0
        );
    }
}