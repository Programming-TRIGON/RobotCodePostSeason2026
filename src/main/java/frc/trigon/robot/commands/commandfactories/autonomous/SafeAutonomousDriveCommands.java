package frc.trigon.robot.commands.commandfactories.autonomous;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.constants.FieldConstants;

import java.util.function.Supplier;

public class SafeAutonomousDriveCommands {
    public static boolean isInAllianceZone() {
        return FieldConstants.isRobotInAllianceZone();
    }

    public static Command THE_MAIN_FUNCTION_IS_HERE(Supplier<FlippablePose2d> targetPose) {
        return new ConditionalCommand(
                //on true
                ,
                //on false
                ,
                //condition
        )
    }
}
