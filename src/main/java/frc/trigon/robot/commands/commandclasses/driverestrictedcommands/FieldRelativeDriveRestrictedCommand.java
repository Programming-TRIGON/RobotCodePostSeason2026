package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.DriveRestriction;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command that drives the robot relative to the field while restricting its movement.
 * See {@link AccelerationRestrictedDrive} for acceleration limiting.
 * See {@link VelocityRestrictedDrive} for velocity limiting.
 * See {@link ZoneRestrictedDrive} for zone restrictions.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public class FieldRelativeDriveRestrictedCommand extends DriveRestrictedCommand {
    /**
     * Drives the robot relative to the field and restricts its movement.
     *
     * @param driveRestrictions Restrictions that restrict the robots movement.
     */
    public FieldRelativeDriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        super(driveRestrictions);
    }

    @Override
    protected Command getDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> restrictedX,
                () -> restrictedY,
                () -> restrictedTheta
        ).repeatedly().asProxy();
    }
}