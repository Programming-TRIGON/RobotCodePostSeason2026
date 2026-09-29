package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    public static Command getResetHopperPositionCommand() {
        return new ConditionalCommand(
                getResetHopperToOpenPositionCommand(),
                getResetHopperToClosePositionCommand(),
                DriverStation::isEnabled
        );
    }

    private static Command getResetHopperToClosePositionCommand() {
        return new InstantCommand(
                RobotContainer.HOPPER::resetToClosePositionMeters
        ).ignoringDisable(true);
    }

    private static Command getResetHopperToOpenPositionCommand() {
        return new StartEndCommand(
                RobotContainer.HOPPER::applyResetPositionVoltage,
                RobotContainer.HOPPER::stop,
                RobotContainer.HOPPER
        ).until(HopperConstants.REED_SWITCH_EVENT);
    }
}
