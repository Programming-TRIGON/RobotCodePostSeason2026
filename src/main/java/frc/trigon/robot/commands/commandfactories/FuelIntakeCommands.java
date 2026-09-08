package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.subsystems.hopper.HopperCommands;
import frc.trigon.robot.subsystems.hopper.HopperConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static LoggedNetworkBoolean SHOULD_INTAKE_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldIntakeDefaultOpen", true);
    public static LoggedNetworkBoolean SHOULD_HOPPER_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldHopperDefaultOpen", true);

    public static Command getPreloadCommand() {
        return new ParallelCommandGroup(
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.PRELOAD)
        ).withTimeout(CommandConstants.PRELOAD_TIMER_SECONDS);
    }

    public static Command getCloseIntakeWhileShootingCommand() {
        return IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_CLOSE);
    }

    public static Command getHopperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.OPEN),
                HopperCommands.getSetTargetStateCommand(HopperConstants.HopperState.CLOSE),
                () -> (SHOULD_HOPPER_DEFAULT_OPEN.getAsBoolean() && (RobotContainer.HOPPER.atState(HopperConstants.HopperState.OPEN) || RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.CLOSE)))
                        || (RobotContainer.HOPPER.atState(HopperConstants.HopperState.OPEN)
                        && !RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.CLOSE))
        );
    }

    public static Command getIntakeDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.OPEN),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE),
                () -> (SHOULD_INTAKE_DEFAULT_OPEN.getAsBoolean() && SHOULD_HOPPER_DEFAULT_OPEN.getAsBoolean())
                        && (RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.OPEN)
                        || RobotContainer.HOPPER.atState(HopperConstants.HopperState.OPEN))
        );
    }
}
