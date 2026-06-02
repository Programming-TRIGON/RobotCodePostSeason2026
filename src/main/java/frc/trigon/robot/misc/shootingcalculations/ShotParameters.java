package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShotParameters(
        double velocity,
        Rotation2d pitch,
        double timeOfFlight
) implements Interpolatable<ShotParameters> {
    @Override
    public ShotParameters interpolate(ShotParameters endValue, double t) {
        return new ShotParameters(
                MathUtil.interpolate(this.velocity, endValue.velocity, t),
                this.pitch.interpolate(endValue.pitch, t),
                MathUtil.interpolate(this.timeOfFlight, endValue.timeOfFlight, t)
        );
    }
}