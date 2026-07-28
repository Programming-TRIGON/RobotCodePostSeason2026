package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FuelIntakeCommands {
    public static LoggedNetworkBoolean SHOULD_INTAKE_STAY_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldIntakeStayOpen", true);

    public static Command getIntakeCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REVERSE_POWERED_OPEN)
                        .until(() -> RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.REVERSE_POWERED_OPEN)),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
        );
    }

    public static Command getIntakeSequenceWhileShootingCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).until(() -> !RobotContainer.SWERVE.isMoving() && !OperatorConstants.INTAKE_TRIGGER.getAsBoolean()),
                IntakeCommands.getFoldForShootingCommand().onlyWhile(() -> !RobotContainer.SWERVE.isMoving() && !OperatorConstants.INTAKE_TRIGGER.getAsBoolean())
        ).repeatedly();
    }
}