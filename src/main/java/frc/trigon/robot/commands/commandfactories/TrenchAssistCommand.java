package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.FieldRelativeRestrictedDriveCommand;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.RestrictedZone;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.ZoneRestrictionsDrive;

public class TrenchAssistCommand {
    private static final double BUMP_LENGTH_METERS = 1.12776;
    private static final double BUMP_WIDTH_METERS = 1.8542;
    private static final double MINIMUM_DISTANCE_METERS_FROM_BUMP_ZONE = 0.1;
    private static final double BRAKING_DISTANCE_FROM_BUMP_ZONE = 0.1;
    private static final BoundingBox RED_LEFT_BUMP_ZONE = new BoundingBox(new Pose2d(new Translation2d(11.821414,5.566918), new Rotation2d()), BUMP_LENGTH_METERS, BUMP_WIDTH_METERS);
    private static final BoundingBox RED_RIGHT_BUMP_ZONE = new BoundingBox(new Pose2d(new Translation2d(11.821414,2.518918), new Rotation2d()), BUMP_LENGTH_METERS, BUMP_WIDTH_METERS);
    private static final BoundingBox BLUE_LEFT_BUMP_ZONE = new BoundingBox(new Pose2d(new Translation2d(4.541774,5.566918), new Rotation2d()), BUMP_LENGTH_METERS, BUMP_WIDTH_METERS);
    private static final BoundingBox BLUE_RIGHT_BUMP_ZONE = new BoundingBox(new Pose2d(new Translation2d(4.541774, 2.518918), new Rotation2d()), BUMP_LENGTH_METERS, BUMP_WIDTH_METERS);

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
        return new RestrictedZone(bumpZone, MINIMUM_DISTANCE_METERS_FROM_BUMP_ZONE, BRAKING_DISTANCE_FROM_BUMP_ZONE);
    }
}