package frc.trigon.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Set;

public class HopperCommands {
    public static LoggedNetworkBoolean SHOULD_HOPPER_RESET_CLOSED = new LoggedNetworkBoolean("/SmartDashboard/ShouldHopperResetClosed", true);

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
        return GeneralCommands.getContinuousConditionalCommand(
                new StartEndCommand(
                        RobotContainer.HOPPER::applyResetClosedPositionVoltage,
                        RobotContainer.HOPPER::resetClosedPosition,
                        RobotContainer.HOPPER
                ),
                new StartEndCommand(
                        RobotContainer.HOPPER::applyResetOpenPositionVoltage,
                        RobotContainer.HOPPER::resetOpenPosition,
                        RobotContainer.HOPPER
                ),
                () -> SHOULD_HOPPER_RESET_CLOSED.get());
    }
}
