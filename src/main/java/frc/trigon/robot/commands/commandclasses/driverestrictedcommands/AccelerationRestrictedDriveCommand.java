package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class AccelerationRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maxTranslationAcceleration;
    private final double maxThetaAcceleration;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter thetaLimiter;

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