package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.DriveRestriction;
import frc.trigon.robot.constants.OperatorConstants;

public abstract class DriveRestrictedCommand extends ParallelCommandGroup {
    private final DriveRestriction[] driveRestrictions;
    protected double
        restrictedX = 0,
        restrictedY = 0,
        restrictedTheta = 0;

    protected DriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        this.driveRestrictions = driveRestrictions;
        addCommands(
                new InstantCommand(this::resetRestrictions),
                new RunCommand(this::setRestrictedOutput),
                getDriveCommand()
        );
    }

    protected abstract Command getDriveCommand();

    protected Translation2d setRelativeDrive(Translation2d shapedTranslation) {
        return shapedTranslation;
    }

    private void resetRestrictions() {
        restrictedX = 0;
        restrictedY = 0;
        restrictedTheta = 0;
        for (DriveRestriction driveRestriction : driveRestrictions)
            driveRestriction.reset();
    }

    private void setRestrictedOutput() {
        Translation2d translation = calculateTargetJoystickTranslation();
        double theta = CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX());

        for (DriveRestriction driveRestriction : driveRestrictions) {
            translation = driveRestriction.applyRestrictionToTranslation(translation);
            theta = driveRestriction.applyRestrictionToTheta(theta);
        }

        restrictedX = translation.getX();
        restrictedY = translation.getY();
        restrictedTheta = theta;
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
}
