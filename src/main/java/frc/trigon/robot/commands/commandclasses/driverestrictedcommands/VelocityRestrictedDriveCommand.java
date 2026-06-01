package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;

public class VelocityRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maxTranslationPower;
    private final double maxThetaPower;

    public VelocityRestrictedDriveCommand(DriveFrame frame, double maxTranslationPower) {
        this(frame, maxTranslationPower, 1.0);
    }

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
