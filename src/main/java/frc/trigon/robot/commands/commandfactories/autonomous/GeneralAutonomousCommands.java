package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;


public class GeneralAutonomousCommands {
    public static boolean isDeliveryFirstAuto = true;

    public static Command getAutonomousScoreCommand() {
        return new ConditionalCommand(
                ShootingCommands.getShootingCommand(),
                new SequentialCommandGroup(
                        SwerveCommands.getFollowPathCommand(() -> scoreOutSideOfAllianceZone()),
                        ShootingCommands.getShootingCommand().withTimeout(AutonomousConstants.SCORING_TIMEOUT_SECONDS)
                        ),
                () -> FieldConstants.isRobotInAllianceZone()
        );
    }

    public static Command getAutonomousDeliveryCommand() {
        return ShootingCommands.getShootingCommand().withTimeout(AutonomousConstants.NORMAL_DELIVERY_TIMEOUT);
    }

    public static Command getAutonomousCollectFromNaturalZoneCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        getGoToFirstIntakePathCommand().deadlineFor(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)),
                        new ParallelCommandGroup(
                                getFirstIntakePathCommand(),
                                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
                        ).withTimeout(AutonomousConstants.NORMAL_INTAKE_TIMEOUT),
                        new InstantCommand(GeneralAutonomousCommands::setDeliveryToFalse)
                ),

                new SequentialCommandGroup(
                        getGoToSecondIntakePathCommand().deadlineFor(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)),
                        new ParallelCommandGroup(
                                getSecondIntakePathCommand(),
                                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
                        ).withTimeout(AutonomousConstants.NORMAL_INTAKE_TIMEOUT)
                ),

                () -> isDeliveryFirstAuto
        );
    }

    private static PathPlannerPath goToPose(Pose2d endWayPoint, PathConstraints pathConstraints, double endPathVelocity, double endHeading, Pose2d... waypoints) {
        double headingAngle = Math.atan2(
                endWayPoint.getY() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getY(),
                endWayPoint.getX() - RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX()
        );

        waypoints = Arrays.stream(waypoints).filter(Objects::nonNull).toArray(Pose2d[]::new);
        Pose2d[] waypointList = new Pose2d[waypoints.length + 2];

        waypointList[0] = new Pose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX(), RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getY(), Rotation2d.fromRadians(headingAngle));
        waypointList[waypointList.length - 1] = new Pose2d(endWayPoint.getX(), endWayPoint.getY(), Rotation2d.fromDegrees(endHeading));

        for (int i = 0; i < waypoints.length; i++) {
            waypointList[i + 1] = waypoints[i];
        }

        List<RotationTarget> rotationTargets = waypoints.length == 2 ? List.of(new RotationTarget(1, Rotation2d.fromDegrees(180)), new RotationTarget(2, Rotation2d.fromDegrees(180))) : List.of();

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(waypointList),
                rotationTargets,
                List.of(),
                List.of(),
                List.of(),
                pathConstraints,
                null,
                new GoalEndState(endPathVelocity, endWayPoint.getRotation()),
                false
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

    private static PathPlannerPath scoreOutSideOfAllianceZone() {
        boolean isRed = Flippable.isRedAlliance();

        double trenchY = FieldConstants.isRight() ? AutonomousConstants.UPPER_TRENCH_Y : AutonomousConstants.LOWER_TRENCH_Y;

        double entranceX = isRed
                ? FieldConstants.FIELD_LENGTH_METERS - AutonomousConstants.BLUE_SCORING_TRENCH_ENTRANCE_X
                : AutonomousConstants.BLUE_SCORING_TRENCH_ENTRANCE_X;

        double exitX = isRed
                ? FieldConstants.FIELD_LENGTH_METERS - AutonomousConstants.BLUE_SCORING_TRENCH_EXIT_X
                : AutonomousConstants.BLUE_SCORING_TRENCH_EXIT_X;

        Rotation2d heading = Rotation2d.fromDegrees(isRed ? 0 : 180);

        Pose2d trenchEntrancePose = new Pose2d(entranceX, trenchY, heading);
        Pose2d trenchExitPose = new Pose2d(exitX, trenchY, heading);

        Pose2d scoringPose = new Pose2d(
                isRed ? FieldConstants.FIELD_LENGTH_METERS - 3.0 : 3.0,
                trenchY,
                Rotation2d.fromDegrees(isRed ? 180 : 0)
        );

        double endHeading = Math.toDegrees(Math.atan2(
                scoringPose.getY() - trenchExitPose.getY(),
                scoringPose.getX() - trenchExitPose.getX()
        ));

        PathPlannerPath path = goToPose(
                scoringPose,
                AutonomousConstants.GOTO_INTAKE_CONSTRAINTS,
                AutonomousConstants.END_PATH_VELOCITY,
                endHeading,
                trenchEntrancePose,
                trenchExitPose
        );

        Logger.recordOutput("Autonomous/scorePath", path.getPathPoses().toArray(new Pose2d[0]));

        return path;
    }

    private static Command getGoToFirstIntakePathCommand() {
        return SwerveCommands.getFollowPathCommand(() -> goToPose(
                FieldConstants.isRight() ? AutonomousConstants.END_LEFT_POSE_FOR_GOTO_FIRST_INTAKE_POSE : AutonomousConstants.END_POSE_FOR_GOTO_FIRST_INTAKE_POSE,
                AutonomousConstants.GOTO_INTAKE_CONSTRAINTS,
                AutonomousConstants.END_GOTO_PATH_VELOCITY,
                FieldConstants.isRight() ? AutonomousConstants.LEFT_FIRST_STARTING_HEADING_ANGLE : AutonomousConstants.FIRST_STARTING_HEADING_ANGLE,
                FieldConstants.isRobotInAllianceZone() ? FieldConstants.isRight() ? AutonomousConstants.RIGHT_TRENCH_ENTRANCE_POSE : AutonomousConstants.LEFT_TRENCH_ENTRANCE_POSE : null,
                FieldConstants.isRobotInAllianceZone() ? FieldConstants.isRight() ? AutonomousConstants.RIGHT_TRENCH_EXIT_POSE : AutonomousConstants.LEFT_TRENCH_EXIT_POSE : null
        ));
    }

    private static Command getGoToSecondIntakePathCommand() {
        return SwerveCommands.getFollowPathCommand(() -> goToPose(
                FieldConstants.isRight() ? AutonomousConstants.END_LEFT_POSE_FOR_GOTO_SECOND_INTAKE_POSE : AutonomousConstants.END_POSE_FOR_GOTO_SECOND_INTAKE_POSE,
                AutonomousConstants.GOTO_INTAKE_CONSTRAINTS,
                AutonomousConstants.END_GOTO_PATH_VELOCITY,
                FieldConstants.isRight() ? AutonomousConstants.LEFT_SECOND_STARTING_HEADING_ANGLE : AutonomousConstants.SECOND_STARTING_HEADING_ANGLE,
                FieldConstants.isRobotInAllianceZone() ? FieldConstants.isRight() ? AutonomousConstants.RIGHT_TRENCH_ENTRANCE_POSE : AutonomousConstants.LEFT_TRENCH_ENTRANCE_POSE : null,
                FieldConstants.isRobotInAllianceZone() ? FieldConstants.isRight() ? AutonomousConstants.RIGHT_TRENCH_EXIT_POSE : AutonomousConstants.LEFT_TRENCH_EXIT_POSE : null
        ));
    }

    private static void setDeliveryToFalse() {
        isDeliveryFirstAuto = false;
    }
}