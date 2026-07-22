package frc.trigon.robot.commands.commandclasses.rumblecommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleChain;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleConfiguration;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.SingleRumble;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RumbleCommands {
    /*
        TODO: Return to zone - Calculate power based on distance, once per second
        TODO: Shooting shift ending - x seconds before shift, once per second increasing power
     */

    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition) {
        return GeneralCommands.runWhen(
                rumbleConfiguration.getRumbleCommand(),
                condition
        ).repeatedly();
    }

    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition, double debounceTimeSeconds) {
        return GeneralCommands.runWhen(
                rumbleConfiguration.getRumbleCommand(),
                condition,
                debounceTimeSeconds
        ).repeatedly();
    }

    public static Command getRumbleOncePerSecondCommand(Supplier<RumbleConfiguration> rumbleConfiguration) {
        return getRumbleOncePerPeriodCommand(rumbleConfiguration, () -> 1);
    }

    public static Command getRumbleOncePerPeriodCommand(Supplier<RumbleConfiguration> rumbleConfiguration, DoubleSupplier periodSeconds) {
        return new SequentialCommandGroup(
                rumbleConfiguration.get().getRumbleCommand(),
                new WaitCommand(periodSeconds.getAsDouble())
        ).repeatedly();
    }

    public enum PremadeRumbles implements RumbleConfiguration {//TODO: Tune on computer with DS
        BIG(new SingleRumble(0.3, 1)),
        SMALL(new SingleRumble(0.2, 0.7)),
        TINY(new SingleRumble(0.1, 1)),
        DOUBLE_TAP(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.3, 1), 0.3),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.3, 1), 0)
        )),
        QUICK_LEFT_THEN_RIGHT(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kLeftRumble), 0),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kRightRumble), 0)
        )),
        QUICK_RIGHT_THEN_LEFT(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kRightRumble), 0),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kLeftRumble), 0)
        ));

        final RumbleConfiguration rumbleConfiguration;

        PremadeRumbles(RumbleConfiguration rumbleConfiguration) {
            this.rumbleConfiguration = rumbleConfiguration;
        }

        @Override
        public Command getRumbleCommand() {
            return rumbleConfiguration.getRumbleCommand();
        }
    }
}
