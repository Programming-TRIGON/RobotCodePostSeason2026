package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShootingMap {
    private static final InterpolatingTreeMap<Double, ShotParameters> INTERPOLATION_MAP = new InterpolatingTreeMap<>(
            (Double start, Double end, Double q) -> (q - start) / (end - start),
            ShotParameters::interpolate
    );

    static {
        // TODO: Replace with actual calibration data.
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
    }

    private static void addPoint(double distanceMeters, double velocityMetersPerSecond, Rotation2d pitch, double timeOfFlightSeconds) {
        INTERPOLATION_MAP.put(distanceMeters, new ShotParameters(velocityMetersPerSecond, pitch, timeOfFlightSeconds));
    }

    public static ShotParameters getInterpolatedParameters(double distanceMeters) {
        if (INTERPOLATION_MAP.get(distanceMeters) == null)
            return new ShotParameters(0, new Rotation2d(), 0);
        return INTERPOLATION_MAP.get(distanceMeters);
    }
}