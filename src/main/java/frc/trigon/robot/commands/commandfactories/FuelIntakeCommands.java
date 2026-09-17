package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.subsystems.hopper.HopperCommands;
import frc.trigon.robot.subsystems.hopper.HopperConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.kicker.KickerCommands;
import frc.trigon.robot.subsystems.kicker.KickerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static LoggedNetworkBoolean SHOULD_INTAKE_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldIntakeDefaultOpen", true);
    public static LoggedNetworkBoolean SHOULD_HOPPER_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldHopperDefaultOpen", true);

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
                        && FuelIntakeCommands.SHOULD_HOPPER_DEFAULT_OPEN.get()
                        && RobotContainer.HOPPER.isPastMinimumPositionForIntakeToOpen()
        );
    }

    public static Command getHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                () -> !FuelIntakeCommands.SHOULD_HOPPER_DEFAULT_OPEN.get()
                        && RobotContainer.INTAKE.isPastMinimumAngleForHopperToClose()
        );
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                getPrepareHopperForIntakeCommand(IntakeConstants.IntakeState.POWERED_OPEN)
        );
    }

    public static Command getPrepareHopperForIntakeCommand(IntakeConstants.IntakeState intakeTargetState) {
        return new ParallelCommandGroup(
                getSetHopperAndIntakeDefaultOpenCommand(),
                getSetIntakeStateWhenHopperSafeCommand(intakeTargetState)
        );
    }

    public static Command getSetHopperAndIntakeDefaultOpenCommand() {
        return new ParallelCommandGroup(
                getSetIntakeDefaultOpenCommand(),
                getSetHopperDefaultOpenCommand()
        );
    }

    public static Command getSetIntakeStateWhenHopperSafeCommand(IntakeConstants.IntakeState targetState) {
        return getWaitUntilSafeForIntakeCommand()
                .andThen(IntakeCommands.getSetTargetStateCommand(targetState));
    }

    public static Command getWaitUntilSafeForIntakeCommand() {
        return new WaitUntilCommand(
                RobotContainer.HOPPER::isPastMinimumPositionForIntakeToOpen
        );
    }

    private static Command getSetHopperDefaultOpenCommand() {
        return new InstantCommand(
                () -> SHOULD_HOPPER_DEFAULT_OPEN.set(true)
        );
    }

    private static Command getSetIntakeDefaultOpenCommand() {
        return new InstantCommand(
                () -> SHOULD_INTAKE_DEFAULT_OPEN.set(true)
        );
    }
}
