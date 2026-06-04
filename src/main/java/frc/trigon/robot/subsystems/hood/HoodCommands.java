package frc.trigon.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.Supplier;

public class HoodCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> RobotContainer.HOOD.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.HOOD),
                "Debugging/HoodTargetAngleDegrees"
        );
    }

    public static Command getAimCommand() {
        return new ExecuteEndCommand(
                RobotContainer.HOOD::aim,
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }

    public static Command getRestCommand() {
        return new StartEndCommand(
                RobotContainer.HOOD::rest,
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }

    public static Command getSetTargetAngleCommand(Supplier<Rotation2d> targetAngleSupplier) {
        return new StartEndCommand(
                () -> RobotContainer.HOOD.setTargetAngle(targetAngleSupplier.get()),
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }
}