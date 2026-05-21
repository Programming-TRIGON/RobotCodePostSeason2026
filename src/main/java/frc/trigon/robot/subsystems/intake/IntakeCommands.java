package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.GearRatioCalculationCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVoltage, targetAngleDegrees) -> RobotContainer.INTAKE.setTargetState(targetVoltage, Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/IntakeTargetVoltage",
                "Debugging/IntakeTargetAngleDegrees"
        );
    }

    public static Command getGearRatioCalculationCommand() {
        return new GearRatioCalculationCommand(
                IntakeConstants.MASTER_ANGLE_MOTOR,
                IntakeConstants.ANGLE_ENCODER,
                0.5,
                RobotContainer.INTAKE
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
