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
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE)
        );
    }

    public static Command getIntakeAndHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSafeOpenIntakeAndHopperCommand(IntakeConstants.IntakeState.OPEN),
                getCloseIntakeAndHopperCommand(IntakeConstants.IntakeState.CLOSE),
                SHOULD_INTAKE_DEFAULT_OPEN
        );
    }


    public static Command getIntakeCommand() {
        return getOpenIntakeAndHopperCommand(IntakeConstants.IntakeState.POWERED_OPEN);
    }

    public static Command getOpenIntakeAndHopperCommand(IntakeConstants.IntakeState targetOpenIntakeState) {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand().andThen(IntakeCommands.getSetTargetStateCommand(targetOpenIntakeState))
        );
    }

    public static Command getSafeOpenIntakeAndHopperCommand(IntakeConstants.IntakeState targetOpenIntakeState) {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand().andThen(IntakeCommands.getSafeSetTargetStateCommand(targetOpenIntakeState))
        );
    }

    public static Command getAutonumousSafeOpenIntakeAndHopperCommand(IntakeConstants.IntakeState targetOpenIntakeState) {
        return new ParallelCommandGroup(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                getWaitUntilSafeForIntakeCommand().andThen(IntakeCommands.getAutonomousSafeSetTargetStateCommand(targetOpenIntakeState))
        );
    }

    public static Command getCloseIntakeAndHopperCommand(IntakeConstants.IntakeState targetCloseIntakeState) {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(targetCloseIntakeState),
                getWaitUntilSafeForHopperCommand()
                        .andThen(HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE))
        );
    }

    private static Command getWaitUntilSafeForIntakeCommand() {
        return new WaitUntilCommand(
                () -> RobotContainer.HOPPER.isPastPosition(HopperConstants.SAFE_POSITION_FOR_INTAKE_METERS)
        );
    }

    private static Command getWaitUntilSafeForHopperCommand() {
        return new WaitUntilCommand(
                () -> RobotContainer.INTAKE.isPastAngle(IntakeConstants.SAFE_ANGLE_FOR_HOPPER)
        );
    }
}
