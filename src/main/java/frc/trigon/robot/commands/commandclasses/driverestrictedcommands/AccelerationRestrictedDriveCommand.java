package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A command that smooths the driver's input by limiting its linear and rotational acceleration.
 */
public class AccelerationRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maximumTranslationAcceleration;
    private final double maximumThetaAcceleration;
    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter thetaLimiter;

    /**
     * Creates a new AccelerationRestrictedDriveCommand.
     *
     * @param frame                          whether the robot is driving relative to the field or to itself.
     * @param maximumTranslationAcceleration maximum linear acceleration.
     * @param maximumThetaAcceleration       maximum rotational acceleration.
     */
    public AccelerationRestrictedDriveCommand(DriveFrame frame, double maximumTranslationAcceleration, double maximumThetaAcceleration) {
        super(frame);
        this.maximumTranslationAcceleration = maximumTranslationAcceleration;
        this.maximumThetaAcceleration = maximumThetaAcceleration;
    }

    @Override
    protected void onInit() {
        translationLimiter = new SlewRateLimiter(maximumTranslationAcceleration);
        thetaLimiter = new SlewRateLimiter(maximumThetaAcceleration);
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        final Translation2d shapedTranslation = new Translation2d(shapedX, shapedY);
        final Translation2d limitedTranslation = limitTranslation(shapedTranslation);
        setRestrictedOutput(
                limitedTranslation.getX(),
                limitedTranslation.getY(),
                thetaLimiter.calculate(shapedTheta)
        );
    }

    private Translation2d limitTranslation(Translation2d shapedTranslation) {
        final double targetMagnitude = shapedTranslation.getNorm();
        final double limitedMagnitude = translationLimiter.calculate(targetMagnitude);
        if (targetMagnitude == 0)
            return Translation2d.kZero;
        return shapedTranslation.times(limitedMagnitude / targetMagnitude);
    }
}