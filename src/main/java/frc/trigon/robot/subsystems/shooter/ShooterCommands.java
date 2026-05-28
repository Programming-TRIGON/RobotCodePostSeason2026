package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SHOOTER::setTargetVelocity,
                false,
                Set.of(RobotContainer.SHOOTER),
                "Debugging/TargetShooterVelocityMetersPerSecond"
        );
    }

    public static Command getSetTargetVelocityMetersPerSecondCommand(DoubleSupplier getSetTargetVelocity) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(getSetTargetVelocity.getAsDouble()),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getShootAtHubCommand() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::aimAtHub,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command shootForDelivery() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::aimForDelivery,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::stop,
                () -> {
                },
                RobotContainer.SHOOTER
        );
    }
}
