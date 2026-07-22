package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.rumblecommands.RumbleCommands;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleConfiguration;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.matchTracker.MatchTracker;

/**
 * A class that contains command factories for assisting the operator/driver's control of the robot, such as indication or debugging commands.
 */
public class OperatorCommands {
    public static Command getRumbleWhenCamerasDisconnectedCommand() {
        return RumbleCommands.getRumbleWhenConditionIsMetCommand(
                RumbleCommands.PremadeRumbles.DOUBLE_TAP,
                () -> !RobotContainer.ROBOT_POSE_ESTIMATOR.hasUpdateFromCameras(),
                OperatorConstants.RUMBLE_WHEN_CAMERAS_DISCONNECTED_DEBOUNCE_TIME_SECONDS
        );
    }

    public static Command getRumbleWhenStillCommand() {
        return GeneralCommands.runWhen(
                RumbleCommands.getRumbleOncePerSecondCommand(() -> RumbleCommands.PremadeRumbles.TINY)
                        .until(OperatorCommands::isMoving),
                OperatorCommands::isStill,
                1
        ).repeatedly();
    }

    public static Command getRumbleToIndicateActiveShiftEndingCommand() {
        return RumbleCommands.getRumbleOncePerSecondCommand(OperatorCommands::determineShiftEndingRumbleConfiguration)
                .onlyWhile(OperatorCommands::shouldRumbleBeforeActiveShiftEnds)
                .repeatedly();
    }

    public static Command getRumbleToIndicateActiveShiftStartingCommand() {
        return RumbleCommands.getRumbleOncePerPeriodCommand(
                        OperatorCommands::determineShiftStartingRumbleConfiguration,
                        OperatorCommands::determineShiftStartingRumbleDelay
                )
                .onlyWhile(OperatorCommands::shouldRumbleBeforeActiveShiftStarts)
                .repeatedly();
    }

    private static boolean isStill() {
        return !RobotContainer.SWERVE.isMoving();
    }

    private static boolean isMoving() {
        return RobotContainer.SWERVE.getSelfRelativeVelocityMetersPerSecond().getNorm() > OperatorConstants.MINIMUM_SWERVE_VELOCITY_METERS_PER_SECOND_TO_STOP_RUMBLING;
    }

    private static RumbleConfiguration determineShiftEndingRumbleConfiguration() {
        if (shouldHeavyRumbleBeforeActiveShiftEnds())
            return RumbleCommands.PremadeRumbles.SMALL;
        return RumbleCommands.PremadeRumbles.BIG;
    }

    private static boolean shouldRumbleBeforeActiveShiftEnds() {
        return MatchTracker.getTimeUntilHubDeactivatesSeconds() <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_ENDS_TO_START_RUMBLING_SECONDS;
    }

    private static boolean shouldHeavyRumbleBeforeActiveShiftEnds() {
        return MatchTracker.getTimeUntilHubDeactivatesSeconds() <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_ENDS_TO_HEAVY_RUMBLE_SECONDS;
    }

    private static boolean shouldRumbleBeforeActiveShiftStarts() {
        final double timeBeforeHubActivatesSeconds = MatchTracker.getTimeUntilHubActivatesSeconds();
        if (timeBeforeHubActivatesSeconds == -1)
            return false;

        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final double timeBeforeActiveShiftToRumble = calculateTimeBeforeActiveShiftToRumble(robotPose.getTranslation());
        return timeBeforeHubActivatesSeconds <= timeBeforeActiveShiftToRumble;
    }

    private static RumbleConfiguration determineShiftStartingRumbleConfiguration() {
        return RumbleCommands.PremadeRumbles.SMALL;
    }

    private static double determineShiftStartingRumbleDelay() {
        return 0;
    }

    private static double calculateTimeBeforeActiveShiftToRumble(Translation2d robotPosition) {
        return 0;
    }
}
