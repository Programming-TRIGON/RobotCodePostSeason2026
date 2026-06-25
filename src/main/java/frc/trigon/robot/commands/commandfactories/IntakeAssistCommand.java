package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.FieldRelativeRestrictedDriveCommand;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.CustomCenterOfRotationDrive;

public class IntakeAssistCommand {
    private static Translation2d INTAKE_CENTER_OF_ROTATION = new Translation2d(0.5,0);

    public static Command getIntakeCenterOfRotationCommand() {
        return new FieldRelativeRestrictedDriveCommand(
                new CustomCenterOfRotationDrive(INTAKE_CENTER_OF_ROTATION)
        );
    }
}