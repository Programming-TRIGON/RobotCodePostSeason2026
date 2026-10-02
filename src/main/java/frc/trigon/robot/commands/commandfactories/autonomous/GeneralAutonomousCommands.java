package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class GeneralAutonomousCommands {
    public static Command getAutonomousScoreCommand() {
        return new SequentialCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                        GeneralAutonomousCommands::getScorePosition,
                        AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                        0,
                        AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                        1000,
                        false,
                        false
                ),
                ShootingCommands.getAutonomousShootingAtHubCommand()
        ).withTimeout(AutonomousConstants.SCORING_TIMEOUT_SECONDS + AutonomousConstants.NORMAL_DRIVE_TIMEOUT);
    }

    public static Command getAutonomousDeliveryCommand() {
        return ShootingCommands.getFixedDeliveryShootingCommand().withTimeout(AutonomousConstants.NORMAL_DELIVERY_TIMEOUT);
    }

/*    public static Command getAutonomousCollectFromNaturalZoneCommand() {
    }*/




    static FlippablePose2d scorePoseInSideNuturalZone() {
        return currentPositionEstimator();
    }

    static FlippablePose2d scorePoseOutSideNuturalZone() {
        return FieldConstants.isRight() ? FieldConstants.IDLE_RIGHT_SCORING_POSE : FieldConstants.IDLE_LEFT_SCORING_POSE;
    }

    static FlippablePose2d getScorePosition(){
        return FieldConstants.isRobotInAllianceZone() ? scorePoseInSideNuturalZone() : scorePoseOutSideNuturalZone();
    }

    private static FlippablePose2d currentPositionEstimator() {
        return new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), false);
    }
}
