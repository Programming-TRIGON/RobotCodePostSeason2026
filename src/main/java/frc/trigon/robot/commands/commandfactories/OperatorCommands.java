package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.rumblecommands.RumbleCommands;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleConfiguration;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.matchTracker.MatchTracker;

/**
 * A class that contains command factories for assisting the operator/driver's control of the robot, such as indication or debugging commands.
 */
public class OperatorCommands {
    public static Command getRumbleCommands() {
        return new ParallelCommandGroup(
                getRumbleWhenCamerasDisconnectedCommand(),
                getRumbleWhenStillCommand(),
                getRumbleToIndicateActiveShiftEndingCommand(),
                getRumbleToIndicateActiveShiftStartingCommand()
        );
    }

    private static Command getRumbleWhenCamerasDisconnectedCommand() {
        return RumbleCommands.getRumbleWhenConditionIsMetCommand(
                RumbleCommands.PremadeRumbles.TRIPLE_TAP,
                () -> !RobotContainer.ROBOT_POSE_ESTIMATOR.hasUpdateFromCameras(),
                OperatorConstants.RUMBLE_WHEN_CAMERAS_DISCONNECTED_DEBOUNCE_TIME_SECONDS
        );
    }

    private static Command getRumbleWhenStillCommand() {
        return GeneralCommands.runWhen(
                RumbleCommands.getRumbleOncePerSecondCommand(() -> RumbleCommands.PremadeRumbles.SMALL)
                        .until(OperatorCommands::isMoving),
                OperatorCommands::isStill,
                1
        ).repeatedly();
    }

    private static Command getRumbleToIndicateActiveShiftEndingCommand() {
        return RumbleCommands.getRumbleOncePerSecondCommand(OperatorCommands::determineShiftEndingRumbleConfiguration)
                .onlyWhile(OperatorCommands::shouldRumbleBeforeActiveShiftEnds)
                .repeatedly();
    }

    private static Command getRumbleToIndicateActiveShiftStartingCommand() {
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
            return RumbleCommands.PremadeRumbles.BIG;
        return RumbleCommands.PremadeRumbles.SMALL;
    }

    private static boolean shouldRumbleBeforeActiveShiftEnds() {
        return MatchTracker.getTimeUntilHubDeactivatesSeconds() <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_ENDS_TO_START_RUMBLING_SECONDS;
    }

    private static boolean shouldHeavyRumbleBeforeActiveShiftEnds() {
        return MatchTracker.getTimeUntilHubDeactivatesSeconds() <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_ENDS_TO_HEAVY_RUMBLE_SECONDS;
    }

    private static boolean shouldRumbleBeforeActiveShiftStarts() {
        final double timeUntilHubActivatesSeconds = MatchTracker.getTimeUntilHubActivatesSeconds();
        if (timeUntilHubActivatesSeconds == -1)
            return false;

        final double timeBeforeActiveShiftToRumble = OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_ONE_RUMBLING_SECONDS;
        return timeUntilHubActivatesSeconds <= timeBeforeActiveShiftToRumble;
    }

    private static RumbleConfiguration determineShiftStartingRumbleConfiguration() {
        final double timeUntilHubActivatesSeconds = MatchTracker.getTimeUntilHubActivatesSeconds();

        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_FOUR_RUMBLING_SECONDS)
            return RumbleCommands.PremadeRumbles.BIG;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_THREE_RUMBLING_SECONDS)
            return RumbleCommands.PremadeRumbles.DOUBLE_TAP;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_TWO_RUMBLING_SECONDS)
            return RumbleCommands.PremadeRumbles.QUICK_RIGHT_THEN_LEFT;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_ONE_RUMBLING_SECONDS)
            return RumbleCommands.PremadeRumbles.TINY;
        return RumbleCommands.PremadeRumbles.NONE;
    }

    private static double determineShiftStartingRumbleDelay() {
        final double timeUntilHubActivatesSeconds = MatchTracker.getTimeUntilHubActivatesSeconds();

        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_FOUR_RUMBLING_SECONDS)
            return 1;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_THREE_RUMBLING_SECONDS)
            return 1;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_TWO_RUMBLING_SECONDS)
            return 1.5;
        if (timeUntilHubActivatesSeconds <= OperatorConstants.TIME_BEFORE_ACTIVE_SHIFT_STARTS_TO_START_LEVEL_ONE_RUMBLING_SECONDS)
            return 2;
        return 0;
    }
}
