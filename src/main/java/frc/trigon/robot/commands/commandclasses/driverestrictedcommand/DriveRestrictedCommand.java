package frc.trigon.robot.commands.commandclasses.driverestrictedcommand;

import edu.wpi.first.math.geometry.Rotation2d;
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
    protected Translation2d robotRelativeCenterOfRotation = Translation2d.kZero; // Stored as a field because the supplier passed to getDriveCommand uses it.
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

    protected Translation2d toFieldRelativeDrive(Translation2d targetTranslation) {
        return targetTranslation;
    }

    protected Translation2d fromFieldRelativeDrive(Translation2d targetTranslation) {
        return targetTranslation;
    }

    protected abstract Command getDriveCommand();

    private void init() {
        restrictedX = 0;
        restrictedY = 0;
        restrictedRotation = 0;
        robotRelativeCenterOfRotation = Translation2d.kZero;
        for (DriveRestriction driveRestriction : driveRestrictions)
            driveRestriction.init();
    }

    private void setRestrictedOutput() {
        Translation2d targetRobotTranslationPower = toFieldRelativeDrive(calculateTargetRobotTranslation());
        double targetRobotRotationPower = CommandConstants.calculateRotationStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX());
        Translation2d targetRobotCenterOfRotation = Translation2d.kZero;

        for (DriveRestriction driveRestriction : driveRestrictions) {
            targetRobotTranslationPower = driveRestriction.applyTranslationRestriction(targetRobotTranslationPower);
            targetRobotRotationPower = driveRestriction.applyRotationRestriction(targetRobotRotationPower);
            targetRobotCenterOfRotation = driveRestriction.applyCenterOfRotationRestriction(targetRobotCenterOfRotation);
        }

        targetRobotTranslationPower = fromFieldRelativeDrive(targetRobotTranslationPower);

        restrictedX = targetRobotTranslationPower.getX();
        restrictedY = targetRobotTranslationPower.getY();
        restrictedRotation = targetRobotRotationPower;
        robotRelativeCenterOfRotation = targetRobotCenterOfRotation;
    }

    /**
     * Calculates the robot's target translation based on driver input.
     *
     * @return the robot's target translation from the joystick, as powers (1,-1)
     */
    private Translation2d calculateTargetRobotTranslation() {
        final Translation2d rawJoystickPosition = getRawJoystickPosition();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getX()),
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getY())
        );
    }

    /**
     * Gets the raw joystick position of the controller.
     *
     * @return the left joysticks position as a power (1,-1), in a Translation2D
     */
    private Translation2d getRawJoystickPosition() {
        final double
                xAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                yAxis = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        return new Translation2d(xAxis, yAxis);
    }
}