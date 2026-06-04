package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.DriveRestriction;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

/**
 * A command that drives the robot relative to itself while restricting its movement.
 * See {@link AccelerationRestrictedDrive} for acceleration limiting.
 * See {@link VelocityRestrictedDrive} for velocity limiting.
 * See {@link ZoneRestrictedDrive} for zone restrictions.
 * All restrictions are applied sequentially, each further restricting the previous result.
 */
public class SelfRelativeDriveRestrictedCommand extends DriveRestrictedCommand {
    /**
     * Drives the robot relative to itself and restricts its movement.
     *
     * @param driveRestrictions Restrictions that restrict the robots movement.
     */
    public SelfRelativeDriveRestrictedCommand(DriveRestriction... driveRestrictions) {
        super(driveRestrictions);
    }

    @Override
    protected Command getDriveCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> restrictedX,
                () -> restrictedY,
                () -> restrictedTheta
        ).repeatedly().asProxy();
    }

    @Override
    protected Translation2d setRelativeDrive(Translation2d shapedTranslation) {
        return shapedTranslation.rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
    }
}