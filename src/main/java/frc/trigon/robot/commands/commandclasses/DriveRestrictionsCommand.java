package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.zonerestricteddrive.ZoneRestriction;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class DriveRestrictionsCommand {
    public interface DriveRestrictions {

    }
    private DriveRestrictions[] driveRestrictions;


    public DriveRestrictionsCommand(DriveRestrictions... driveRestrictions) {

    }

   private Translation2d applyRestriction(Translation2d targetTranslation, DriveRestrictions driveRestrictions) {
        final double
   }


    private Translation2d calculateTargetJoystickTranslation() {
        final Translation2d rawJoystickPosition = getRawJoystickPosition();

        return new Translation2d(
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getX()),
                CommandConstants.calculateDriveStickAxisValue(rawJoystickPosition.getY())
        );
    }

    private Translation2d getFieldRelativeJoystickPosition() {
        return getRawJoystickPosition().rotateBy(Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    private Translation2d getRawJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();
        return new Translation2d(joystickY, joystickX);
    }
}