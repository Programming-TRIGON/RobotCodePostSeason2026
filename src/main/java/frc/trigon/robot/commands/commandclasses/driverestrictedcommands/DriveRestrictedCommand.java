package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public abstract class DriveRestrictedCommand extends ParallelCommandGroup {

    public enum DriveFrame {
        FIELD_RELATIVE,
        SELF_RELATIVE
    }

    private final DriveFrame frame;
    private volatile double restrictedX = 0;
    private volatile double restrictedY = 0;
    private volatile double restrictedTheta = 0;

    protected DriveRestrictedCommand(DriveFrame frame) {
        this.frame = frame;
        addCommands(
                buildCalculationCommand(),
                buildDriveCommand()
        );
    }

    protected abstract void restrict(double shapedX, double shapedY, double shapedTheta);

    protected void onInit() {}

    protected final void setRestrictedOutput(double x, double y, double theta) {
        this.restrictedX = x;
        this.restrictedY = y;
        this.restrictedTheta = theta;
    }

    protected final DriveFrame getFrame() {
        return frame;
    }

    private Command buildCalculationCommand() {
        return new RunCommand(this::readAndRestrict);
    }

    private void readAndRestrict() {
        final double rawTheta = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        final double shapedX = calculateTargetJoystickTranslation().getX();
        final double shapedY = calculateTargetJoystickTranslation().getY();
        final double shapedTheta = CommandConstants.calculateRotationStickAxisValue(rawTheta);

        if (frame == DriveFrame.SELF_RELATIVE) {
            final Translation2d selfRel = new Translation2d(shapedX, shapedY)
                    .rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
            restrict(selfRel.getX(), selfRel.getY(), shapedTheta);
            return;
        }

        final Translation2d fieldRel = new Translation2d(shapedX, shapedY)
                .rotateBy(Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
        restrict(fieldRel.getX(), fieldRel.getY(), shapedTheta);
    }

    private Command buildDriveCommand() {
        final Command drive = switch (frame) {
            case FIELD_RELATIVE -> SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> restrictedX,
                    () -> restrictedY,
                    () -> restrictedTheta
            );
            case SELF_RELATIVE -> SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> restrictedX,
                    () -> restrictedY,
                    () -> restrictedTheta
            );
        };
        return drive.repeatedly().asProxy();
    }

    private Translation2d getRawJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();
        return new Translation2d(joystickX, joystickY);
    }

    private Translation2d calculateTargetJoystickTranslation() {
        final double
                rawXValue = getRawJoystickPosition().getX(),
                rawYValue = getRawJoystickPosition().getY();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawXValue),
                CommandConstants.calculateDriveStickAxisValue(rawYValue)
        );
    }
}