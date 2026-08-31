package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShootingMap {
    private static final InterpolatingTreeMap<Double, ShotParameters> HUB_MAP = new InterpolatingTreeMap<>(
            (Double start, Double end, Double q) -> (q - start) / (end - start),
            ShotParameters::interpolate
    );

    private static final InterpolatingTreeMap<Double, ShotParameters> DELIVERY_MAP = new InterpolatingTreeMap<>(
            (Double start, Double end, Double q) -> (q - start) / (end - start),
            ShotParameters::interpolate
    );

    static {
        addHubPoints();
        addDeliveryPoints();
    }

    private static void addHubPoints() {
        addPoint(HUB_MAP, 4.87760115561, 9.5, Rotation2d.fromDegrees(37), 1.45);
        addPoint(HUB_MAP, 3.836144086, 8.75, Rotation2d.fromDegrees(28), 1.42);
        addPoint(HUB_MAP, 2.686144, 8.1, Rotation2d.fromDegrees(25), 1.25);
        addPoint(HUB_MAP, 2.472544086, 7.7, Rotation2d.fromDegrees(25), 1.18);
        addPoint(HUB_MAP, 2.041144086, 7.2, Rotation2d.fromDegrees(24), 1.07);
        addPoint(HUB_MAP, 1.666144086, 6.8, Rotation2d.fromDegrees(22), 1.07);
    }

    private static void addDeliveryPoints() {
        addPoint(DELIVERY_MAP, 2.50, 5.2, Rotation2d.fromDegrees(26.5), 0.68);
        addPoint(DELIVERY_MAP, 3.00, 5.81, Rotation2d.fromDegrees(27), 0.75);
        addPoint(DELIVERY_MAP, 4.00, 7.03, Rotation2d.fromDegrees(29), 0.92);
        addPoint(DELIVERY_MAP, 5.00, 8.3, Rotation2d.fromDegrees(31), 1.09);
        addPoint(DELIVERY_MAP, 6.00, 9.70, Rotation2d.fromDegrees(34), 1.29);
        addPoint(DELIVERY_MAP, 7.00, 11.46, Rotation2d.fromDegrees(37), 1.51);
    }

    private static void addPoint(InterpolatingTreeMap<Double, ShotParameters> map, double distanceMeters, double velocityMetersPerSecond, Rotation2d pitch, double timeOfFlightSeconds) {
        map.put(distanceMeters, new ShotParameters(velocityMetersPerSecond, pitch, timeOfFlightSeconds));
    }

    public static ShotParameters getInterpolatedParameters(double distanceMeters, boolean isDelivery) {
        InterpolatingTreeMap<Double, ShotParameters> activeMap = isDelivery ? DELIVERY_MAP : HUB_MAP;

        if (activeMap.get(distanceMeters) == null)
            return new ShotParameters(0, new Rotation2d(), 0);
        return activeMap.get(distanceMeters);
    }
}