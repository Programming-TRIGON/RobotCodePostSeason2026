package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MatchTrackerCommands {
    public static Command getForceGameDataCommand(char gameData) {
        return new InstantCommand(() -> MatchTracker.forceGameData(gameData));
    }
}
