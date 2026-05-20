package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetWheelVoltage, targetIntakeVoltage) -> RobotContainer.INTAKE.setTargetState(targetWheelVoltage, targetIntakeVoltage),
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/WheelTargetVoltage",
                "Debugging/IntakeTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetStateCommand(double targetWheelVoltage, double targetIntakeVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetWheelVoltage, targetIntakeVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}
