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
        addPoint(HUB_MAP, 0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(HUB_MAP, 0, 0, Rotation2d.fromDegrees(0), 0);
    }
    
    private static void addDeliveryPoints() {
        addPoint(DELIVERY_MAP, 0, 0, Rotation2d.fromDegrees(0), 0);
        addPoint(DELIVERY_MAP, 0, 0, Rotation2d.fromDegrees(0), 0);
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