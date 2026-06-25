package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.OperatorConstants;

public class MatchTrackerCommands {
    public static Command getEnableOverrideGameDataCommand() {
        return new InstantCommand(() -> ShootingCommands.enableOverrideGameData());
    }

    public static Command getDisableOverrideGameDataCommand() {
        return new InstantCommand(() -> ShootingCommands.disableOverrideGameData());
    }

    public static Command getRumbleCommand() {
        return new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(MatchTrackerConstants.RUMBLE_TIME_SECONDS, MatchTrackerConstants.RUMBLE_POWER));
    }
}