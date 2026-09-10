package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.FuelIntakeCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;

import java.util.Set;

public class HopperCommands {
    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE),
                FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN
        );
    }

    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.HOPPER::setTargetPositionMeters,
                false,
                Set.of(RobotContainer.HOPPER),
                "Debugging/HopperTargetPositionMeters"
        );
    }

    public static Command getSetTargetStateCommand(HopperConstants.HopperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.HOPPER.setTargetState(targetState),
                RobotContainer.HOPPER::stop,
                RobotContainer.HOPPER
        );
    }
}
