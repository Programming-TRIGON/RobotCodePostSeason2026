package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;

/**
 * A command that caps the linear and rotational velocity of the robot.
 */
public class VelocityRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maximumTranslationVelocity;
    private final double maximumThetaVelocity;

    /**
     * Creates a new VelocityRestrictedDriveCommand.
     *
     * @param maximumTranslationVelocity maximum linear velocity, has to be between 1 and 0.
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maximumTranslationVelocity) {
        this(frame, maximumTranslationVelocity, 1.0);
    }

    /**
     * Creates a new VelocityRestrictedDriveCommand.
     *
     * @param maximumTranslationVelocity maximum linear velocity, needs to be between 1 and 0.
     * @param maximumThetaVelocity       maximum rotational velocity, needs to be between 1 and 0.
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maximumTranslationVelocity, double maximumThetaVelocity) {
        super(frame);
        this.maximumTranslationVelocity = maximumTranslationVelocity;
        this.maximumThetaVelocity = maximumThetaVelocity;
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        final double norm = Math.hypot(shapedX, shapedY);
        final double cappedTheta = MathUtil.clamp(shapedTheta, -maximumThetaVelocity, maximumThetaVelocity);

        if (norm > maximumTranslationVelocity) {
            final double scale = maximumTranslationVelocity / norm;
            setRestrictedOutput(shapedX * scale, shapedY * scale, cappedTheta);
        } else
            setRestrictedOutput(shapedX, shapedY, cappedTheta);
    }
}