package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Smooths the driver's input by limiting how fast each axis can change per second. Each axis has
 * its own slew-rate limiter, reset in {@link #onInit()} so each scheduling starts fresh with no
 * carry-over state.
 * <p>
 * Note: per-axis limiting can slightly distort circular stick motion under aggressive limits
 * (the limiters work independently on X and Y). For most uses this is fine.
 */
public class AccelerationRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maxTranslationAcceleration;
    private final double maxThetaAcceleration;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter thetaLimiter;

    /**
     * @param maxTranslationAcceleration maximum change in translation power per second
     *                                   (e.g. 5.0 = full reverse to full forward in 0.4 s)
     * @param maxThetaAcceleration       same idea for rotation
     */
    public AccelerationRestrictedDriveCommand(DriveFrame frame,
                                              double maxTranslationAcceleration,
                                              double maxThetaAcceleration) {
        super(frame);
        this.maxTranslationAcceleration = maxTranslationAcceleration;
        this.maxThetaAcceleration = maxThetaAcceleration;
    }

    @Override
    protected void onInit() {
        xLimiter = new SlewRateLimiter(maxTranslationAcceleration);
        yLimiter = new SlewRateLimiter(maxTranslationAcceleration);
        thetaLimiter = new SlewRateLimiter(maxThetaAcceleration);
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
