package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;

public class MatchTrackerCommands {
    public static Command getEnableOverrideGameDataCommand() {
        return new InstantCommand(ShootingCommands::enableOverrideGameData).ignoringDisable(true);
    }

    public static Command getDisableOverrideGameDataCommand() {
        return new InstantCommand(ShootingCommands::disableOverrideGameData).ignoringDisable(true);
    }
}