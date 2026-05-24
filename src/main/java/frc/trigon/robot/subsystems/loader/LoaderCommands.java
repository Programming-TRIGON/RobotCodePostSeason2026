package frc.trigon.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class LoaderCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.LOADER::setTargetVoltage,
                false,
                Set.of(RobotContainer.LOADER),
                "Debugging/LoaderTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(LoaderConstants.LoaderState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.LOADER.setTargetState(targetState),
                RobotContainer.LOADER::stop,
                RobotContainer.LOADER
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.LOADER.setTargetVoltage(targetVoltage),
                RobotContainer.LOADER::stop,
                RobotContainer.LOADER
        );
    }
}
