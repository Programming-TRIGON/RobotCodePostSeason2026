package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetIntakeVoltage, targetIntakeArmAngle) -> RobotContainer.INTAKE.setTargetState(targetIntakeVoltage, Rotation2d.fromDegrees(targetIntakeArmAngle)),
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/IntakeTargetVoltage",
                "Debugging/IntakeArmTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}
