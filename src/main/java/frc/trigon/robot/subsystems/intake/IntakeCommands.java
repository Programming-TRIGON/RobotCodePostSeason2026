package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.commands.GearRatioCalculationCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.FuelIntakeCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class IntakeCommands {
    private static final BooleanEvent IS_STUCK_ON_HOPPER = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> !RobotContainer.INTAKE.atTargetState()
            && RobotContainer.INTAKE.isIntakeStuckOnHopper()).debounce(0.2);

    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN),
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

    public static Command getAutonomousSafeSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new SequentialCommandGroup(
                getSetTargetStateCommand(targetState).until(() -> IS_STUCK_ON_HOPPER.getAsBoolean() && targetState.targetAngle.equals(IntakeConstants.MINIMUM_ANGLE)),
                new SequentialCommandGroup(
                        getSetTargetStateCommand(IntakeConstants.IntakeState.ASSIST_OPEN).withTimeout(1),
                        getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE).withTimeout(0.2)
                ).repeatedly().until(RobotContainer.INTAKE::atTargetState)
        ).repeatedly();
    }

    public static Command getSafeSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new SequentialCommandGroup(
                getSetTargetStateCommand(targetState).until(() -> IS_STUCK_ON_HOPPER.getAsBoolean() && targetState.targetAngle.equals(IntakeConstants.MINIMUM_ANGLE)),
                getSetTargetStateCommand(IntakeConstants.IntakeState.ASSIST_OPEN).until(RobotContainer.INTAKE::atTargetState)
        ).repeatedly();
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