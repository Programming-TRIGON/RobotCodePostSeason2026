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
        // Format: Distance (Meters), Velocity (m/s), Pitch (Degrees), Time of Flight (Seconds)
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(0, 0, Rotation2d.fromDegrees(0), 0);
    }

    private static void addPoint(double distance, double velocity, Rotation2d pitchDegrees, double timeOfFlight) {
        INTERPOLATION_MAP.put(distance, new ShotParameters(velocity, pitchDegrees, timeOfFlight));
    }

    public static ShotParameters getInterpolatedParameters(double distanceMeters) {
        if (INTERPOLATION_MAP.get(distanceMeters) == null)
            return new ShotParameters(0, new Rotation2d(), 0); // Failsafe for empty map
        return INTERPOLATION_MAP.get(distanceMeters);
    }
}