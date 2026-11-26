package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.hopper.HopperCommands;
import frc.trigon.robot.subsystems.hopper.HopperConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static LoggedNetworkBoolean SHOULD_INTAKE_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldIntakeDefaultOpen", true);
    public static LoggedNetworkBoolean SHOULD_HOPPER_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldHopperDefaultOpen", true);

    public static Command getIntakeCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REVERSE_POWERED_OPEN)
                        .until(() -> RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.REVERSE_POWERED_OPEN)),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
        );
    }

    public static Command getIntakeDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                () -> FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.get()
                        && RobotContainer.HOPPER.isPastMinimumPositionForIntakeToOpen()
        );
    }

    public static Command getHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                () -> !FuelIntakeCommands.SHOULD_HOPPER_DEFAULT_OPEN.get()
                        && !FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.get()
                        && RobotContainer.INTAKE.isPastMinimumAngleForHopperToClose()
        );
    }

    public static Command getOpenIntakeWhenHopperReadyCommand(IntakeConstants.IntakeState targetState) {
        return new ParallelCommandGroup(
                getSetIntakeDefaultOpenCommand(),
                getSetHopperDefaultOpenCommand(),
                getSetIntakeOpenStateWhenHopperReadyCommand(targetState)
        );
    }

    private static Command getSetIntakeOpenStateWhenHopperReadyCommand(IntakeConstants.IntakeState targetState) {
        return new SequentialCommandGroup(
                getWaitUntilSafeForIntakeToOpenCommand(),
                IntakeCommands.getSetTargetStateCommand(targetState)
        );
    }

    public static Command getCloseIntakeWhileShootingCommand() {
        return IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE);
    }

    private static Command getWaitUntilSafeForIntakeToOpenCommand() {
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
