package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.subsystems.hopper.HopperCommands;
import frc.trigon.robot.subsystems.hopper.HopperConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.kicker.KickerCommands;
import frc.trigon.robot.subsystems.kicker.KickerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.function.BooleanSupplier;

public class FuelIntakeCommands {
    public static LoggedNetworkBoolean SHOULD_INTAKE_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldIntakeDefaultOpen", true);

    public static Command getPreloadCommand() {
        return new ParallelCommandGroup(
                KickerCommands.getSetTargetStateCommand(KickerConstants.KickerState.PRELOAD)
        ).withTimeout(CommandConstants.PRELOAD_TIMER_SECONDS);
    }

    public static Command getCloseIntakeWhileShootingCommand() {
        return IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE);
    }

    public static Command getIntakeDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                () -> FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.get()
                        && RobotContainer.HOPPER.isPastPosition(HopperConstants.MINIMUM_POSITION_FOR_INTAKE_METERS)
        );
    }

    public static Command getHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                () -> !FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.get()
                        && RobotContainer.INTAKE.isPastAngle(IntakeConstants.MINIMUM_ANGLE_FOR_HOPPER)
        );
    }

    public static Command getIntakeCommand(HopperConstants.HopperState hopperState, IntakeConstants.IntakeState intakeState, BooleanSupplier isSafe) {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(hopperState),
                getWaitUntilSafeForIntakeCommand().andThen(getSafeOrUnsafeSetTargetStateCommand(intakeState, isSafe))
        );
    }

    public static Command getSafeOpenIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand().andThen(IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN))
        );
    }

    public static Command getPoweredCloseIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE),
                getWaitUntilSafeForHopperCommand().andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE))
        );
    }

    public static Command getCloseIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                getWaitUntilSafeForHopperCommand().andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE))
        );
    }

    public static Command getWaitUntilSafeForIntakeCommand() {
        return new WaitUntilCommand(
                () -> RobotContainer.HOPPER.isPastPosition(HopperConstants.MINIMUM_POSITION_FOR_INTAKE_METERS)
        );
    }

    public static Command getWaitUntilSafeForHopperCommand() {
        return new WaitUntilCommand(
                () -> RobotContainer.INTAKE.isPastAngle(IntakeConstants.MINIMUM_ANGLE_FOR_HOPPER)
        );
    }

    private static Command getSafeOrUnsafeSetTargetStateCommand(IntakeConstants.IntakeState intakeState, BooleanSupplier isSafe) {
        return new ConditionalCommand(
                IntakeCommands.getSafeSetTargetStateCommand(intakeState),
                IntakeCommands.getSetTargetStateCommand(intakeState),
                isSafe
        );
    }
}
