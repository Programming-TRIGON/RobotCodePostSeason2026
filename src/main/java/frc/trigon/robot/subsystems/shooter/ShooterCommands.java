package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static LoggedNetworkBoolean SHOULD_SHOOTER_DEFAULT_ON = new LoggedNetworkBoolean("/SmartDashboard/ShouldShooterDefaultOn", true);
    private static final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();

    public static Command getBoostedDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SHOOTER::setBoostedTargetVelocity,
                false,
                Set.of(RobotContainer.SHOOTER),
                "Debugging/ShooterTargetVelocityMetersPerSecond"
        ).withTimeout(ShooterConstants.SHOOTING_START_VELOCITY_BOOST_DURATION_SECONDS)
                .andThen(getDebuggingCommand());
    }

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

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::stop,
                () -> {
                },
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocityCommand(DoubleSupplier targetVelocityMetersPerSecond) {
        return new RunCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityMetersPerSecond.getAsDouble()),
                RobotContainer.SHOOTER
        );
    }

    public static Command getBoostedSetTargetVelocityCommand(DoubleSupplier targetVelocityMetersPerSecond) {
        return new RunCommand(
                () -> RobotContainer.SHOOTER.setBoostedTargetVelocity(targetVelocityMetersPerSecond.getAsDouble()),
                RobotContainer.SHOOTER
        ).withTimeout(ShooterConstants.SHOOTING_START_VELOCITY_BOOST_DURATION_SECONDS)
                .andThen(getSetTargetVelocityCommand(targetVelocityMetersPerSecond));
    }

    public static Command getAimForShootingCommand() {
        return getSetTargetVelocityCommand(() -> shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond());
    }

    public static Command getBoostedAimForShootingCommand() {
        return getBoostedSetTargetVelocityCommand(() -> shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond());
    }
}
