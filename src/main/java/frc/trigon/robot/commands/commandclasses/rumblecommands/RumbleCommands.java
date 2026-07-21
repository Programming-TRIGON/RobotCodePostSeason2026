package frc.trigon.robot.commands.commandclasses.rumblecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleConfiguration;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;

import java.util.function.BooleanSupplier;

public class RumbleCommands {
    /*
    TODO
        Cameras disconnected
        Return to zone
        Shooting shift ending
        Still counter
     */

    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition) {
        return GeneralCommands.runWhen(
                rumbleConfiguration.getRumbleCommand(),
                condition
        );
    }

    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition, double debounceTimeSeconds) {
        return GeneralCommands.runWhen(
                rumbleConfiguration.getRumbleCommand(),
                condition,
                debounceTimeSeconds
        );
    }
}
