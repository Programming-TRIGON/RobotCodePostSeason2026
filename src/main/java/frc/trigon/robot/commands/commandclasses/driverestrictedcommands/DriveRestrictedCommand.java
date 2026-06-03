package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * Abstract base for driving commands that modify the driver's input before sending it to the swerve.
 */
public abstract class DriveRestrictedCommand extends ParallelCommandGroup {
    private final DriveFrame frame;
    private double restrictedX = 0;
    private double restrictedY = 0;
    private double restrictedTheta = 0;

    /**
     * Creates a new DriveRestrictedDriveCommand.
     *
     * @param frame whether the robot is driving relative to the field or to itself.
     */
    protected DriveRestrictedCommand(DriveFrame frame) {
        this.frame = frame;
        addCommands(
                new InstantCommand(() -> {
                    restrictedX = 0;
                    restrictedY = 0;
                    restrictedTheta = 0;
                    onInit();
                }),
                getRestrictDriveCommand(),
                getDriveCommand()
        );
    }

    protected abstract void restrict(double shapedX, double shapedY, double shapedTheta);

    protected void onInit() {
    }

    protected final void setRestrictedOutput(double x, double y, double theta) {
        this.restrictedX = x;
        this.restrictedY = y;
        this.restrictedTheta = theta;
    }

    protected final DriveFrame getFrame() {
        return frame;
    }

    private Command getRestrictDriveCommand() {
        return new RunCommand(this::restrictDrive);
    }

    private void restrictDrive() {
        final double rawTheta = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        final Translation2d shapedTranslation = calculateTargetJoystickTranslation();
        final double
                shapedX = shapedTranslation.getX(),
                shapedY = shapedTranslation.getY(),
                shapedTheta = CommandConstants.calculateRotationStickAxisValue(rawTheta);

        if (frame == DriveFrame.SELF_RELATIVE) {
            final Translation2d selfRelative = new Translation2d(shapedX, shapedY)
                    .rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
            restrict(selfRelative.getX(), selfRelative.getY(), shapedTheta);
            return;
        }

        restrict(shapedX, shapedY, shapedTheta);
    }

    private Command getDriveCommand() {
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
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        return new Translation2d(joystickX, joystickY);
    }

    private Translation2d calculateTargetJoystickTranslation() {
        final Translation2d rawPosition = getRawJoystickPosition();
        final double
                rawXValue = rawPosition.getX(),
                rawYValue = rawPosition.getY();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawXValue),
                CommandConstants.calculateDriveStickAxisValue(rawYValue)
        );
    }

    /**
     * An enum that is used to represent whether the robot is driving relative to the field or relative to itself.
     * The enum is used to decide which method of driving gets restricted.
     */
    public enum DriveFrame {
        FIELD_RELATIVE,
        SELF_RELATIVE
    }
}