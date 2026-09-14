package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.FuelIntakeCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;

import java.util.Set;

public class HopperCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.HOPPER::setTargetPositionMeters,
                false,
                Set.of(RobotContainer.HOPPER),
                "Debugging/HopperTargetPositionMeters"
        );
    }

    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                FuelIntakeCommands.getSafeOpenIntakeAndHopperCommand(),
                FuelIntakeCommands.getCloseIntakeAndHopperCommand(),
                FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN
        );
    }

    public static Command getSetTargetStateCommand(HopperConstants.HopperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.HOPPER.setTargetState(targetState),
                RobotContainer.HOPPER::stop,
                RobotContainer.HOPPER
        );
    }

    public static Command getWaitUntilSafeForIntakeCommand() {
        return new WaitUntilCommand(
                () -> RobotContainer.HOPPER.isPastPosition(HopperConstants.MINIMUM_POSITION_FOR_INTAKE_METERS)
        );
    }

    public static Command getResetHopperCommand() {
        return new StartEndCommand(
                RobotContainer.HOPPER::applyResetPositionVoltage,
                RobotContainer.HOPPER::resetPosition,
                RobotContainer.HOPPER
        ).ignoringDisable(true);
    }
}
