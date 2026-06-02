package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Smooths the driver's input by limiting how fast each axis can change per second.
 * Each axis has its own slew-rate limiter.
 */
public class AccelerationRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maximumTranslationAcceleration;
    private final double maximumThetaAcceleration;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter thetaLimiter;

    /**
     * Creates a new AccelerationRestrictedDriveCommand.
     *
     * @param maximumTranslationAcceleration maximum change in translation power per second
     * @param maximumThetaAcceleration       maximum change in rotation power per second
     */
    public AccelerationRestrictedDriveCommand(DriveFrame frame,
                                              double maximumTranslationAcceleration,
                                              double maximumThetaAcceleration) {
        super(frame);
        this.maximumTranslationAcceleration = maximumTranslationAcceleration;
        this.maximumThetaAcceleration = maximumThetaAcceleration;
    }

    @Override
    protected void onInit() {
        xLimiter = new SlewRateLimiter(maximumTranslationAcceleration);
        yLimiter = new SlewRateLimiter(maximumTranslationAcceleration);
        thetaLimiter = new SlewRateLimiter(maximumThetaAcceleration);
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        setRestrictedOutput(
                xLimiter.calculate(shapedX),
                yLimiter.calculate(shapedY),
                thetaLimiter.calculate(shapedTheta)
        );
    }
}