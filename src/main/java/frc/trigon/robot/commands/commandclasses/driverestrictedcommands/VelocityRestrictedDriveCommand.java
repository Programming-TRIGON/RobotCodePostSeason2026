package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;


/**
 * Built on {@link DriveRestrictedCommand}
 *
 * Caps the magnitude of the driver's translation input. Rotation can optionally be capped too.
 */
public class VelocityRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maximumTranslationPower;
    private final double maximumThetaPower;

    /**
     * @param maximumTranslationPower maximum translation magnitude as a fraction of full power, in (0, 1]
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maximumTranslationPower) {
        this(frame, maximumTranslationPower, 1.0);
    }

    /**
     * Creates a new VelocityRestrictedDriveCommand.
     *
     * @param maximumTranslationPower maximum translation magnitude as a fraction of full power, in (0, 1]
     * @param maximumThetaPower       maximum rotation magnitude as a fraction of full power, in (0, 1]
     */
    public VelocityRestrictedDriveCommand(DriveFrame frame, double maximumTranslationPower, double maximumThetaPower) {
        super(frame);
        this.maximumTranslationPower = maximumTranslationPower;
        this.maximumThetaPower = maximumThetaPower;
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        final double norm = Math.hypot(shapedX, shapedY);
        final double cappedTheta = MathUtil.clamp(shapedTheta, -maximumThetaPower, maximumThetaPower);

        if (norm > maximumTranslationPower) {
            final double scale = maximumTranslationPower / norm;
            setRestrictedOutput(shapedX * scale, shapedY * scale, cappedTheta);
        } else {
            setRestrictedOutput(shapedX, shapedY, cappedTheta);
        }
    }
}