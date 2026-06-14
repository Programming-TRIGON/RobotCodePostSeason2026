package frc.trigon.robot.commands.commandclasses.driverestrictedcommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.DriveRestriction;
import frc.trigon.robot.constants.OperatorConstants;

/**
 * An abstract class for a command that drives the robot while restricting its movement.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public abstract class DriveRestrictedCommand extends ParallelCommandGroup {
    private final DriveRestriction[] driveRestrictions;
    protected Translation2d centerOfRotation = Translation2d.kZero;
    protected double
            restrictedX = 0,
            restrictedY = 0,
            restrictedRotation = 0;

    /**
     * Constructs a command that drives the robot and restricts its movement.
     *
     * @param driveRestrictions the restrictions to apply, in order. Each receives the output of the previous one.
     */
    protected DriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        this.driveRestrictions = driveRestrictions;
        addCommands(
                new InstantCommand(this::init),
                new RunCommand(this::setRestrictedOutput),
                getDriveCommand()
        );
    }

    protected abstract Command getDriveCommand();

    protected abstract Translation2d selfRelativeDrive(Translation2d targetTranslation);

    private void init() {
        restrictedX = 0;
        restrictedY = 0;
        restrictedRotation = 0;
        centerOfRotation = Translation2d.kZero;
        for (DriveRestriction driveRestriction : driveRestrictions)
            driveRestriction.init();
    }

    private void setRestrictedOutput() {
        Translation2d targetRobotTranslation = selfRelativeDrive(calculateTargetJoystickTranslation());
        double targetRobotRotation = CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX());
        Translation2d targetRobotCenterOfRotation = Translation2d.kZero;

        for (DriveRestriction driveRestriction : driveRestrictions) {
            targetRobotTranslation = driveRestriction.ApplyTranslationRestriction(targetRobotTranslation);
            targetRobotRotation = driveRestriction.ApplyRotationRestriction(targetRobotRotation);
            targetRobotCenterOfRotation = driveRestriction.getCenterOfRotation();
        }

        restrictedX = targetRobotTranslation.getX();
        restrictedY = targetRobotTranslation.getY();
        restrictedRotation = targetRobotRotation;
        centerOfRotation = targetRobotCenterOfRotation;
    }

    /**
     * Calculates the target joystick translation based on driver input.
     *
     * @return the target translation
     */
    private Translation2d calculateTargetJoystickTranslation() {
        final Translation2d rawJoystickPosition = getRawJoystickPosition();
        final double
                rawXValue = rawJoystickPosition.getX(),
                rawYValue = rawJoystickPosition.getY();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawXValue),
                CommandConstants.calculateDriveStickAxisValue(rawYValue)
        );
    }

    /**
     * Gets the raw joystick position of the controller.
     *
     * @return the joysticks position as a Translation2D
     */
    private Translation2d getRawJoystickPosition() {
        final double
                xAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                yAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        return new Translation2d(xAxis, yAxis);
    }
}
