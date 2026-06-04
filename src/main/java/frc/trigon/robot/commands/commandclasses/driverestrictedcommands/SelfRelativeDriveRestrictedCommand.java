package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.DriveRestriction;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command that drives the robot relative to itself while restricting its movement.
 * Acceleration restriction {@link AccelerationRestrictedDrive}.
 * Velocity restriction {@link VelocityRestrictedDrive}.
 * Zone restriction {@link ZoneRestrictedDrive}.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public class SelfRelativeDriveRestrictedCommand extends ParallelCommandGroup {
    private final DriveRestriction[] driveRestrictions;
    private double
            restrictedX,
            restrictedY,
            restrictedTheta;

    /**
     * Drives the robot relative to itself and restricts its movement.
     *
     * @param driveRestrictions Restrictions that restrict the robots movement.
     */
    public SelfRelativeDriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        this.driveRestrictions = driveRestrictions;
        addCommands(
                new RunCommand(this::setRestrictedOutput),
                getDriveCommand()
        );
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

    final Command getDriveCommand() {
        final Command drive = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> restrictedX,
                () -> restrictedY,
                () -> restrictedTheta
        );
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
        ).rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle()).unaryMinus();
    }
}