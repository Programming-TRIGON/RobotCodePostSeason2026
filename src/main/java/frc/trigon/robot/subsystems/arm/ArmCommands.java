package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Set;

public class ArmCommands {
    public static final LoggedNetworkBoolean SHOULD_ARM_DEFAULT_OPEN = new LoggedNetworkBoolean("/SmartDashboard/ShouldArmDefaultOpen");
    public static final LoggedNetworkNumber ARM_VOLTAGE = new LoggedNetworkNumber("/SmartDashboard/ArmVoltage", 1);
    public static boolean DSBSA = false;

    public static Command getDefaultCommand() {
        return getSetTargetStateCommand(ArmConstants.ArmState.CLOSE);
    }

    public static Command getMoveArmUpCommand() {
        return new StartEndCommand(
                () -> RobotContainer.ARM.sysIDDrive(ARM_VOLTAGE.get()),
                () -> RobotContainer.ARM.stop(),
                RobotContainer.ARM
        );
    }

    public static Command getMoveArmDownCommand() {
        return new StartEndCommand(
                () -> RobotContainer.ARM.sysIDDrive(-ARM_VOLTAGE.get()),
                () -> RobotContainer.ARM.stop(),
                RobotContainer.ARM
        );
    }

    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> RobotContainer.ARM.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.HOOD),
                "Debugging/ArmTargetAngleDegrees"
        );
    }

    public static Command getResetArmCommand() {
        return new StartEndCommand(
                RobotContainer.ARM::resetTargetVoltage,
                RobotContainer.ARM::resetPosition,
                RobotContainer.ARM
        ).ignoringDisable(true);
    }

    public static Command getSetTargetStateCommand(ArmConstants.ArmState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                RobotContainer.ARM::stop,
                RobotContainer.ARM
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.ARM::stop,
                () -> {
                },
                RobotContainer.ARM
        );
    }
}