package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.FieldRelativeRestrictedDriveCommand;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.RestrictedZone;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.ZoneRestrictionsDrive;

public class TrenchAssistCommand {
    private static double minimumDistanceMetersFromBumpZone = 0.1;
    private static double brakingDistanceFromBumpZone = 0.1;
    private static BoundingBox RED_LEFT_BUMP_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox RED_RIGHT_BUMP_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox BLUE_LEFT_BUMP_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox BLUE_RIGHT_BUMP_ZONE = new BoundingBox(new Translation2d(), new Translation2d());

    public static Command getTrenchAssistCommand() {
        return new FieldRelativeRestrictedDriveCommand(
                new ZoneRestrictionsDrive(
                        true,
                        getBumpRestrictedZone(RED_LEFT_BUMP_ZONE),
                        getBumpRestrictedZone(RED_RIGHT_BUMP_ZONE),
                        getBumpRestrictedZone(BLUE_LEFT_BUMP_ZONE),
                        getBumpRestrictedZone(BLUE_RIGHT_BUMP_ZONE))
        );
    }

    private static RestrictedZone getBumpRestrictedZone(BoundingBox bumpZone) {
        return new RestrictedZone(bumpZone, minimumDistanceMetersFromBumpZone, brakingDistanceFromBumpZone);
    }
}