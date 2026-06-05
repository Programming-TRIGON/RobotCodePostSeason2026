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
        addPoint(HUB_MAP, 2, 6.2, Rotation2d.fromDegrees(62), 0.76);
        addPoint(HUB_MAP, 3, 6.85, Rotation2d.fromDegrees(60), 0.92);
        addPoint(HUB_MAP, 4, 7.75, Rotation2d.fromDegrees(58), 1.06);
        addPoint(HUB_MAP, 5, 8.4, Rotation2d.fromDegrees(57), 1.16);
        addPoint(HUB_MAP, 6, 9.2, Rotation2d.fromDegrees(57), 1.3);
    }

    private static void addDeliveryPoints() { //TODO: calibrate numbers, these are example numbers gotten from desmos
        addPoint(DELIVERY_MAP, 3.00913, 5, Rotation2d.fromDegrees(46), 0.86624);
        addPoint(DELIVERY_MAP, 4.16762, 6, Rotation2d.fromDegrees(45), 0.98223);
        addPoint(DELIVERY_MAP, 5.18138, 6.76, Rotation2d.fromDegrees(44), 1.065478);
        addPoint(DELIVERY_MAP, 6.36517, 7.56, Rotation2d.fromDegrees(43), 1.15121);
        addPoint(DELIVERY_MAP, 8.66555, 8.95, Rotation2d.fromDegrees(40), 1.264);
        addPoint(DELIVERY_MAP, 10.49719, 10, Rotation2d.fromDegrees(37), 1.31457);
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