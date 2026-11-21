package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.awt.*;
import java.util.List;


public class GeneralAutonomousCommands {
    public static boolean isDeliveryFirstAuto = true;;

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
        return ShootingCommands.getShootingCommand().withTimeout(AutonomousConstants.NORMAL_DELIVERY_TIMEOUT);
    }

    public static Command getAutonomousCollectFromNaturalZoneCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        getGoToFirstIntakePathCommand(),
                        new ParallelCommandGroup(
                                getFirstIntakePathCommand(),
                                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
                        ).withTimeout(AutonomousConstants.NORMAL_INTAKE_TIMEOUT),
                        new InstantCommand(GeneralAutonomousCommands::setDeliveryToFalse)
                ),

                new SequentialCommandGroup(
                        getGoToSecondIntakePathCommand(),
                        new ParallelCommandGroup(
                                getSecondIntakePathCommand(),
                                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
                        ).withTimeout(AutonomousConstants.NORMAL_INTAKE_TIMEOUT)
                ),

                () -> isDeliveryFirstAuto
        );
    }

    private static PathPlannerPath goToIntakePath(Pose2d endWayPoint, PathConstraints pathConstraints, double endPathVelocity, double endHeading, Pose2d... waypoints) {
        double headingAngle = Math.atan2(
                endWayPoint.getY() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getY(),
                endWayPoint.getX() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX()
        );

        Pose2d[] waypointList = new Pose2d[waypoints.length + 2];

        waypointList[0] = new Pose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX(), RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getY(), Rotation2d.fromRadians(headingAngle));
        waypointList[waypointList.length - 1] = new Pose2d(endWayPoint.getX(), endWayPoint.getY(), Rotation2d.fromDegrees(endHeading));

        for (int i = 0; i < waypoints.length; i++) {
            waypointList[i + 1] = waypoints[i];
        }

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(waypointList),
                pathConstraints,
                null,
                new GoalEndState(endPathVelocity, endWayPoint.getRotation())
        );

        path.preventFlipping = true;

        Logger.recordOutput("Autonomous/currentGoToIntakePath", path.getPathPoses().toArray(new Pose2d[0]));

        return path;
    }

    private static Command getFirstIntakePathCommand() {
        Pose2d rightStartIntakePose = new Pose2d(AutonomousConstants.FIRST_STARTING_X_POSITION, AutonomousConstants.FIRST_STARTING_Y_POSITION, Rotation2d.fromDegrees(AutonomousConstants.FIRST_STARTING_HEADING_ANGLE));
        Pose2d rightEndIntakePose = new Pose2d(AutonomousConstants.FIRST_END_X_POSITION, AutonomousConstants.FIRST_END_Y_POSITION, Rotation2d.fromDegrees(AutonomousConstants.FIRST_END_HEADING_ANGLE));
        Pose2d leftStartIntakePose = FieldConstants.mirror(rightStartIntakePose, Rotation2d.fromDegrees(AutonomousConstants.LEFT_FIRST_STARTING_HEADING_ANGLE));
        Pose2d leftEndIntakePose = FieldConstants.mirror(rightEndIntakePose, Rotation2d.fromDegrees(AutonomousConstants.LEFT_FIRST_END_HEADING_ANGLE));

        List<Waypoint> FIRST_INTAKE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                FieldConstants.isRight() ? leftStartIntakePose : rightStartIntakePose,
                FieldConstants.isRight() ? leftEndIntakePose : rightEndIntakePose
        );

        PathPlannerPath path = new PathPlannerPath(
                FIRST_INTAKE_WAYPOINTS,
                AutonomousConstants.INTAKE_CONSTRAINTS,
                null,
                new GoalEndState(AutonomousConstants.END_PATH_VELOCITY, FieldConstants.isRight() ? leftEndIntakePose.getRotation() : rightEndIntakePose.getRotation())
        );

        path.preventFlipping = true;

        Logger.recordOutput("Autonomous/firstIntakePath", path.getPathPoses().toArray(new Pose2d[0]));

        return SwerveCommands.getFollowPathCommand(() -> path);
    }

    private static Command getSecondIntakePathCommand() {
        Pose2d rightStartIntakePose = new Pose2d(AutonomousConstants.SECOND_STARTING_X_POSITION, AutonomousConstants.SECOND_STARTING_Y_POSITION, Rotation2d.fromDegrees(AutonomousConstants.SECOND_STARTING_HEADING_ANGLE));
        Pose2d rightEndIntakePose = new Pose2d(AutonomousConstants.SECOND_END_X_POSITION, AutonomousConstants.SECOND_END_Y_POSITION, Rotation2d.fromDegrees(AutonomousConstants.SECOND_END_HEADING_ANGLE));
        Pose2d leftStartIntakePose = FieldConstants.mirror(rightStartIntakePose, Rotation2d.fromDegrees(AutonomousConstants.LEFT_SECOND_STARTING_HEADING_ANGLE));
        Pose2d leftEndIntakePose = FieldConstants.mirror(rightEndIntakePose, Rotation2d.fromDegrees(AutonomousConstants.LEFT_SECOND_END_HEADING_ANGLE));

        List<Waypoint> SECOND_INTAKE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                FieldConstants.isRight() ? leftStartIntakePose : rightStartIntakePose,
                FieldConstants.isRight() ? leftEndIntakePose : rightEndIntakePose
        );

        PathPlannerPath path = new PathPlannerPath(
                SECOND_INTAKE_WAYPOINTS,
                AutonomousConstants.INTAKE_CONSTRAINTS,
                null,
                new GoalEndState(AutonomousConstants.END_PATH_VELOCITY, FieldConstants.isRight() ? leftEndIntakePose.getRotation() : rightEndIntakePose.getRotation())
        );

        path.preventFlipping = true;

        Logger.recordOutput("Autonomous/secondIntakePath", path.getPathPoses().toArray(new Pose2d[0]));

        return SwerveCommands.getFollowPathCommand(() -> path);
    }

    private static Command getGoToFirstIntakePathCommand() {
        return SwerveCommands.getFollowPathCommand(() -> goToIntakePath(
                FieldConstants.isRight() ? AutonomousConstants.END_LEFT_POSE_FOR_GOTO_FIRST_INTAKE_POSE : AutonomousConstants.END_POSE_FOR_GOTO_FIRST_INTAKE_POSE,
                AutonomousConstants.GOTO_INTAKE_CONSTRAINTS,
                AutonomousConstants.END_GOTO_PATH_VELOCITY,
                FieldConstants.isRight() ? AutonomousConstants.LEFT_FIRST_STARTING_HEADING_ANGLE : AutonomousConstants.FIRST_STARTING_HEADING_ANGLE
        ));
    }

    private static Command getGoToSecondIntakePathCommand() {
        return SwerveCommands.getFollowPathCommand(() -> goToIntakePath(
                FieldConstants.isRight() ? AutonomousConstants.END_LEFT_POSE_FOR_GOTO_SECOND_INTAKE_POSE : AutonomousConstants.END_POSE_FOR_GOTO_SECOND_INTAKE_POSE,
                AutonomousConstants.GOTO_INTAKE_CONSTRAINTS,
                AutonomousConstants.END_GOTO_PATH_VELOCITY,
                FieldConstants.isRight() ? AutonomousConstants.LEFT_SECOND_STARTING_HEADING_ANGLE : AutonomousConstants.SECOND_STARTING_HEADING_ANGLE
        ));
    }

    static FlippablePose2d scorePoseInSideNuturalZone() {
        return currentPositionEstimator();
    }

    static FlippablePose2d scorePoseOutSideNuturalZone() {
        return FieldConstants.isRight() ? FieldConstants.IDLE_RIGHT_SCORING_POSE : FieldConstants.IDLE_LEFT_SCORING_POSE;
    }

    static FlippablePose2d getScorePosition() {
        return FieldConstants.isRobotInAllianceZone() ? scorePoseInSideNuturalZone() : scorePoseOutSideNuturalZone();
    }

    private static FlippablePose2d currentPositionEstimator() {
        return new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), false);
    }

    private static void setDeliveryToFalse() {
        isDeliveryFirstAuto = false;
    }
}