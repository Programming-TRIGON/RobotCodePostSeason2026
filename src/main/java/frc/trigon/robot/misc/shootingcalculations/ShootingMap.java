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

    private static void addHubPoints() { //TODO: calibrate real robot numbers
        addPoint(HUB_MAP, 1.55, 6.8, Rotation2d.fromDegrees(25), 1);
        addPoint(HUB_MAP, 2.18, 7.5, Rotation2d.fromDegrees(25), 1.09);
        addPoint(HUB_MAP, 2.58, 8.13, Rotation2d.fromDegrees(25), 1.15);
        addPoint(HUB_MAP, 3.81, 8.8, Rotation2d.fromDegrees(30), 1.4);
        addPoint(HUB_MAP, 5.20, 9.8, Rotation2d.fromDegrees(38), 1.4);
    }

    private static void addDeliveryPoints() { //TODO: calibrate real robot numbers
        addPoint(DELIVERY_MAP, 3, 5.65, Rotation2d.fromDegrees(20), 0.540);
        addPoint(DELIVERY_MAP, 5, 7.1, Rotation2d.fromDegrees(30), 0.8);
        addPoint(DELIVERY_MAP, 7, 8.7, Rotation2d.fromDegrees(33), 1);
        addPoint(DELIVERY_MAP, 10, 10.6, Rotation2d.fromDegrees(40), 1.34);

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