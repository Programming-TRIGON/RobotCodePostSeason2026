package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

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

    public static Command getSetTargetStateCommand(HopperConstants.HopperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.HOPPER.setTargetState(targetState),
                RobotContainer.HOPPER::stop,
                RobotContainer.HOPPER
        );
    }

    public static Command getManualResetHopperToClosePositionCommand() {
        return new StartEndCommand(
                () -> RobotContainer.HOPPER.applyResetPositionVoltage(HopperConstants.RESET_TO_CLOSE_POSITION_VOLTAGE),
                RobotContainer.HOPPER::resetToClosePositionMeters,
                RobotContainer.HOPPER
        ).ignoringDisable(true);
    }

    public static Command getResetHopperToReedSwitchCommand() {
        return new FunctionalCommand(
                () -> {},
                () -> RobotContainer.HOPPER.applyResetPositionVoltage(HopperConstants.RESET_TO_REED_SWITCH_POSITION_VOLTAGE),
                interrupted -> RobotContainer.HOPPER.stop(),
                HopperConstants.REED_SWITCH::getBinaryValue,
                RobotContainer.HOPPER
        );
    }

    public static Command getResetHopperPositionToCloseCommand() {
        return new InstantCommand(
                RobotContainer.HOPPER::resetToClosePositionMeters
        ).ignoringDisable(true);
    }
}
