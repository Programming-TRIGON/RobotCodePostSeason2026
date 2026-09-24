package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
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

    public static Command getResetHopperPositionCommand() {
        return new ConditionalCommand(
                getResetHopperToOpenCommand(),
                getResetHopperToClosePositionCommand(),
                DriverStation::isEnabled
        );
    }

    public static Command getResetHopperToClosePositionCommand() {
        return new InstantCommand(
                RobotContainer.HOPPER::resetToClosePositionMeters
        );
    }

    public static Command getResetHopperToOpenCommand() {
        return new FunctionalCommand(
                () -> {
                },
                RobotContainer.HOPPER::applyResetPositionVoltage,
                interrupted -> RobotContainer.HOPPER.stop(),
                HopperConstants.REED_SWITCH::getBinaryValue,
                RobotContainer.HOPPER
        );
    }
}
