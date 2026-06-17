package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 20-second autonomous period at the start of each match.
 */
public class GeneralAutonomousCommands {
    public static Command getDeliveryCommand(AutonomousGenerator.AutonomousState previousState, Supplier<Double> collectionTimeout) {
//        return new ParallelDeadlineGroup(
//                getDriveToFuelInNeutralZoneCommand(
//                        AutonomousGenerator.SHOULD_SHOOT_PRELOAD.getAsBoolean() && previousState == null,
//                        collectionTimeout.get(),
//                        previousState == null,
//                        getIntakingPoseInNeutralZone(previousState),
//                        true
//
//                ),
//                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN),
//                getDeliverWhileDrivingCommand()
//        );
        return Commands.none();
    }

    public static Command getCollectFromNeutralZoneCommand(AutonomousGenerator.AutonomousState previousState, double collectionTimeout) {
        return new ParallelDeadlineGroup(
                getSafeDriveToIntakeFromNeutralPath(),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)
        );
    }

    public static Command getScoreCommand(AutonomousGenerator.AutonomousState nextState, double timeout) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                                () -> AutonomousConstants.SHOOTING_POSE,
                                AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                                0,
                                AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                                1000,
                                false
                        ),
                        new RunCommand(() -> Logger.recordOutput("Autonomous/TargetScoringPose", getScoringPose(nextState).get()))),
                ShootingCommands.getShootingCommand().withTimeout(AutonomousConstants.SCORING_TIMEOUT_SECONDS)
        ).withTimeout(timeout + AutonomousConstants.NORMAL_DRIVE_TIMEOUT);
    }

    public static Command getTimedScoreCommand(double timeout) {
        return ShootingCommands.getAutonomousShootingCommand().withTimeout(timeout);
    }

    public static Command getCollectFromDepotCommand(boolean shootWhileDriving, double collectionTimeout) {
        return SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                () -> FieldConstants.DEPOT_POSITION,
                AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                0,
                AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                shootWhileDriving ? 1000 : 0,
                false
        ).alongWith(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN));
    }

    private static boolean isAfterTrenchX() {
        final Pose2d currentRobotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        return currentRobotPose.getX() < FieldConstants.TRENCH_ALLIANCE_ENTRY_AUTONOMOUS_X;
    }

    private static Command getSafeDriveToIntakeFromNeutralPath() {
        return new SequentialCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(() ->AutonomousConstants.NEUTRAL_INTAKE_POSE, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                getIntakeFromNeutralPath()
        );
    }

    private static Command getIntakeFromNeutralPath() {
        try {
            PathPlannerPath neutralIntakePath = PathPlannerPath.fromPathFile("Neutral Collect");
            return SwerveCommands.getFollowPathCommand(() -> neutralIntakePath);
        } catch (Exception e) {
            System.out.println("Couldn't find path");
            return Commands.none();
        }
    }

    static FlippablePose2d getScoringPose(AutonomousGenerator.AutonomousState nextState) {
        if (nextState == AutonomousGenerator.AutonomousState.COLLECT_FROM_DEPOT)
            return FieldConstants.DEPOT_POSITION;
        return SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_IDEAL_SHOOTING_POSITION : FieldConstants.LEFT_IDEAL_SHOOTING_POSITION;
    }

    /**
     * Creates a command that resets the pose estimator's pose to the starting pose of the given autonomous as long as the robot is not enabled.
     *
     * @param autoName the name of the autonomous
     * @return a command that resets the robot's pose estimator pose to the start position of the given autonomous
     */
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        ).ignoringDisable(true);
    }

    /**
     * Gets the starting position of the target PathPlanner autonomous.
     *
     * @param autoName the name of the autonomous group
     * @return the staring pose of the autonomous
     */
    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonFlippedAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final FlippablePose2d flippedAutoStartPose = new FlippablePose2d(nonFlippedAutoStartPose, true);
            return flippedAutoStartPose.get();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}