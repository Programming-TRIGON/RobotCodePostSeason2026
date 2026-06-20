package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.constants.OperatorConstants;

public class MatchTrackerCommands {
    public static Command enableOverrideGameDataCommand() {
        return new InstantCommand(() -> MatchTracker.overrideGameData());
    }

    public static Command disableOverrideGameDataCommand() {
        return new InstantCommand(() -> MatchTracker.disableOverrideGameData());
    }

    public static Command getRumbleCommand() {
        return new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(MatchTrackerConstants.RUMBLE_TIME_IN_SECONDS, MatchTrackerConstants.RUMBLE_POWER));
    }
}
