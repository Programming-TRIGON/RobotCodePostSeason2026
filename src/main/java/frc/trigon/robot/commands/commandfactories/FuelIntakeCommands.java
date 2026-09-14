package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
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
                getWaitUntilSafeForIntakeCommand().andThen(IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN)),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                SHOULD_INTAKE_DEFAULT_OPEN
        );
    }

    public static Command getHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForHopperCommand().andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE)),
                SHOULD_INTAKE_DEFAULT_OPEN
        );
    }

    public static Command getIntakeCommand() {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand()
                        .andThen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN))
        );
    }

    public static Command getSafeIntakeCommand() {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand()
                        .andThen(IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN))
        );
    }

    public static Command getSafeOpenIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand()
                        .andThen(IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN))
        );
    }

    public static Command getPoweredCloseIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE),
                getWaitUntilSafeForHopperCommand()
                        .andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE))
        );
    }

    public static Command getCloseIntakeAndHopperCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                getWaitUntilSafeForHopperCommand()
                        .andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE))
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
}
