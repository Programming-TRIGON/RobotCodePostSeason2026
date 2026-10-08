package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static LoggedNetworkBoolean SHOULD_SHOOTER_DEFAULT_ON = new LoggedNetworkBoolean("/SmartDashboard/ShouldShooterDefaultOn", true);

    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SHOOTER::setTargetVelocity,
                false,
                Set.of(RobotContainer.SHOOTER),
                "Debugging/ShooterTargetVelocityMetersPerSecond"
        );
    }

    public static Command getShooterDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSetTargetVelocityCommand(() -> ShooterConstants.DEFAULT_SHOOTER_VELOCITY_METERS_PER_SECOND),
                getStopCommand(),
                () -> SHOULD_SHOOTER_DEFAULT_ON.get()

        );
    }

    public static Command getSetTargetVelocityCommand(DoubleSupplier targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityMetersPerSecond.getAsDouble()),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getAimForShootingCommand() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::aimForShooting,
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
