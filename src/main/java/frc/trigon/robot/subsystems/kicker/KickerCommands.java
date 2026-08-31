package frc.trigon.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class KickerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.KICKER::setTargetVelocity,
                false,
                Set.of(RobotContainer.KICKER),
                "Debugging/KickerTargetVelocityMetersPerSecond"
        );
    }

    public static Command getSetTargetStateCommand(KickerConstants.KickerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.KICKER.setTargetState(targetState),
                RobotContainer.KICKER::stop,
                RobotContainer.KICKER
        );
    }
}
