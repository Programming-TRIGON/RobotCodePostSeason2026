package frc.trigon.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.Shooter.Shooter;

import java.util.Set;

public class ShooterCommand {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SHOOTER::getSetTargetVelocity,
                false,
                Set.of(RobotContainer.SHOOTER),
                "Debugging/TargetShooterVelocityMetersPerSecond"
        );
    }

    public static Command getSetVelocityMeterPerSecondCommand(double velocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.getSetTargetVelocity(velocityMetersPerSecond),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }
}
