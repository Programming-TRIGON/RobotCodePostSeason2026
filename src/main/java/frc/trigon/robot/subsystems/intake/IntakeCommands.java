package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.GearRatioCalculationCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.FuelIntakeCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;

import java.util.Set;

public class IntakeCommands {
    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSetTargetStateCommand(IntakeConstants.IntakeState.OPEN),
                getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN
        );
    }

    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVoltage, targetAngleDegrees) -> {
                    RobotContainer.INTAKE.setTargetVoltage(targetVoltage);
                    RobotContainer.INTAKE.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees));
                },
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/IntakeTargetVoltage",
                "Debugging/IntakeTargetAngleDegrees"
        );
    }

    public static Command getSafeSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new ConditionalCommand(getAssistIntakeOpenCommand(),
                getSetTargetStateCommand(targetState),
                () -> targetState.targetAngle == IntakeConstants.MINIMUM_ANGLE
                        && !RobotContainer.INTAKE.atAngle(IntakeConstants.MINIMUM_ANGLE)
                        && RobotContainer.INTAKE.shouldAssistIntakeOpen());
    }

    public static Command getAssistIntakeOpenCommand() {
        return getSetTargetStateCommand(IntakeConstants.IntakeState.ASSIST_OPEN);
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
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
}