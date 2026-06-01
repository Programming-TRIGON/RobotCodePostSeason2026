package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;

/**
 * Caps the magnitude of the driver's translation input. Rotation can optionally be capped too.
 * <p>
 * Useful for "score-prep" / "fine alignment" modes where you want to slow the robot without
 * forcing the driver to ease off the stick.
 */
public class VelocityRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maxTranslationPower;
    private final double maxThetaPower;

    /**
     * @param maxTranslationPower maximum translation magnitude as a fraction of full power, in (0, 1]
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maxTranslationPower) {
        this(frame, maxTranslationPower, 1.0);
    }

    /**
     * @param maxTranslationPower maximum translation magnitude as a fraction of full power, in (0, 1]
     * @param maxThetaPower       maximum rotation magnitude as a fraction of full power, in (0, 1]
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maxTranslationPower, double maxThetaPower) {
        super(frame);
        this.maxTranslationPower = maxTranslationPower;
        this.maxThetaPower = maxThetaPower;
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        final double norm = Math.hypot(shapedX, shapedY);
        final double cappedTheta = MathUtil.clamp(shapedTheta, -maxThetaPower, maxThetaPower);

        if (norm > maxTranslationPower) {
            final double scale = maxTranslationPower / norm;
            setRestrictedOutput(shapedX * scale, shapedY * scale, cappedTheta);
        } else {
            setRestrictedOutput(shapedX, shapedY, cappedTheta);
        }
    }
}
