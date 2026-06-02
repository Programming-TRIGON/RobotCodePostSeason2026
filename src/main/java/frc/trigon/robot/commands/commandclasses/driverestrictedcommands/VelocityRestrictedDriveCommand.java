package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;

public class VelocityRestrictedDriveCommand extends DriveRestrictedCommand {
    private final double maximumTranslationPower;
    private final double maximumThetaPower;

    public VelocityRestrictedDriveCommand(DriveFrame frame, double maximumTranslationPower) {
        this(frame, maximumTranslationPower, 1.0);
    }

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