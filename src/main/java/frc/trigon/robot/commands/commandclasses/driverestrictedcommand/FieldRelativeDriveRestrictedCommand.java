package frc.trigon.robot.commands.commandclasses.driverestrictedcommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.DriveRestriction;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command that drives the robot relative to the field while restricting its movement.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public class FieldRelativeDriveRestrictedCommand extends DriveRestrictedCommand {
    /**
     * Constructs a command that drives the robot relative to the field and restricts its movement.
     *
     * @param driveRestrictions Restricts the robot's movement.
     */
    public FieldRelativeDriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        super(driveRestrictions);
    }

    @Override
    protected Command getDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> restrictedX,
                () -> restrictedY,
                () -> restrictedRotation
        ).asProxy();
    }
}