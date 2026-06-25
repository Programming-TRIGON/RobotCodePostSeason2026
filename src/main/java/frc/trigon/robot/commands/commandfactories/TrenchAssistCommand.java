package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.FieldRelativeRestrictedDriveCommand;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.RestrictedZone;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.ZoneRestrictionsDrive;

public class TrenchAssistCommand {
    private static double minimumDistanceMetersFromZone = 0.1;
    private static double brakingDistanceFromZone = 0.1;
    private static BoundingBox RED_LEFT_TRENCH_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox RED_RIGHT_TRENCH_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox BLUE_LEFT_TRENCH_ZONE = new BoundingBox(new Translation2d(), new Translation2d());
    private static BoundingBox BLUE_RIGHT_TRENCH_ZONE = new BoundingBox(new Translation2d(), new Translation2d());

    public static Command trenchAssistCommand() {
        return new FieldRelativeRestrictedDriveCommand(
                new ZoneRestrictionsDrive(
                        true,
                        getTrenchRestrictedZone(RED_LEFT_TRENCH_ZONE),
                        getTrenchRestrictedZone(RED_RIGHT_TRENCH_ZONE),
                        getTrenchRestrictedZone(BLUE_LEFT_TRENCH_ZONE),
                        getTrenchRestrictedZone(BLUE_RIGHT_TRENCH_ZONE))
        );
    }

    private static RestrictedZone getTrenchRestrictedZone(BoundingBox trenchZone) {
        return new RestrictedZone(trenchZone, minimumDistanceMetersFromZone, brakingDistanceFromZone);
    }
}