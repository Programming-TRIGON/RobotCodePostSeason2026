package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.LocalADStarAK;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.commands.commandfactories.autonomous.AutonomousGenerator;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.json.simple.parser.ParseException;

import java.awt.*;
import java.io.IOException;
import java.util.List;

/**
 * A class that contains the constants and configurations for everything related to the 15-second autonomous period at the start of the match.
 */
public class AutonomousConstants {
    public static final String DEFAULT_AUTO_NAME = "DefaultAutoName";
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = RobotHardwareStats.isSimulation()
            ? 0.3
            : 0.5;//TODO: Calibrate

    public final static double
            END_PATH_VELOCITY = 0.0,
            END_GOTO_PATH_VELOCITY = 1.5,

            FIRST_STARTING_X_POSITION = 8.25,
            FIRST_STARTING_Y_POSITION = 1.3,
            FIRST_END_X_POSITION = 8.25,
            FIRST_END_Y_POSITION = 4.0,

            SECOND_STARTING_X_POSITION = 10.0,
            SECOND_STARTING_Y_POSITION = 1.3,
            SECOND_END_X_POSITION = 10.0,
            SECOND_END_Y_POSITION = 4.0,

            FIRST_STARTING_HEADING_ANGLE = 90.0,
            SECOND_STARTING_HEADING_ANGLE = 90.0,
            FIRST_END_HEADING_ANGLE = 90.0,
            SECOND_END_HEADING_ANGLE = 90.0,

            LEFT_FIRST_STARTING_HEADING_ANGLE = -90.0,
            LEFT_SECOND_STARTING_HEADING_ANGLE = -90.0,
            LEFT_FIRST_END_HEADING_ANGLE = -90.0,
            LEFT_SECOND_END_HEADING_ANGLE = -90.0,

            ENTER_RIGHT_TRENCH_X_POSITION = 12.8,
            ENTER_RIGHT_TRENCH_Y_POSITION = 7.48,
            EXIT_RIGHT_TRENCH_X_POSITION = 11.1,
            EXIT_RIGHT_TRENCH_Y_POSITION = 7.48,

            ENTER_LEFT_TRENCH_X_POSITION = 12.8,
            ENTER_LEFT_TRENCH_Y_POSITION = 0.65,
            EXIT_LEFT_TRENCH_X_POSITION = 11.1,
            EXIT_LEFT_TRENCH_Y_POSITION = 0.65,

            HEADING_ANGLE_FROM_ALLIANCE_ZONE = 180,
            HEADING_ANGLE_TO_ALLIANCE_ZONE = -180;

    public static final double
            BLUE_SCORING_TRENCH_ENTRANCE_X = 5.223,
            BLUE_SCORING_TRENCH_EXIT_X = 4.000,
            LOWER_TRENCH_Y = 0.64,
            UPPER_TRENCH_Y = FieldConstants.FIELD_WIDTH_METERS - LOWER_TRENCH_Y;

    private static final double
            IDLE_SCORING_X_POSITION = 3,
            IDLE_SCORING_Y_POSITION = 0.65;

    public static final Pose2d
            IDLE_RIGHT_SCORING_POSE = new Pose2d(IDLE_SCORING_X_POSITION, IDLE_SCORING_Y_POSITION, Rotation2d.kZero),
            IDLE_LEFT_SCORING_POSE = FieldConstants.mirror(IDLE_RIGHT_SCORING_POSE, Rotation2d.kZero);


    private final static double
            INTAKE_MAX_VELOCITY_MPS = 1.5,
            INTAKE_MAX_ACCELERATION_MPSSQ = 1,
            INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC = 1 * Math.PI,
            INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = 1 * Math.PI;

    private final static double
            GOTO_INTAKE_MAX_VELOCITY_MPS = 5.0,
            GOTO_INTAKE_MAX_ACCELERATION_MPSSQ = 4.0,
            GOTO_INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC = 2 * Math.PI,
            GOTO_INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = 4 * Math.PI;

    public static final PathConstraints INTAKE_CONSTRAINTS = new PathConstraints(
            INTAKE_MAX_VELOCITY_MPS,
            INTAKE_MAX_ACCELERATION_MPSSQ,
            INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC,
            INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ
    );

    public static final PathConstraints GOTO_INTAKE_CONSTRAINTS = new PathConstraints(
            GOTO_INTAKE_MAX_VELOCITY_MPS,
            GOTO_INTAKE_MAX_ACCELERATION_MPSSQ,
            GOTO_INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC,
            GOTO_INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ
    );

    public static Pose2d END_POSE_FOR_GOTO_FIRST_INTAKE_POSE = new Pose2d(FIRST_STARTING_X_POSITION, FIRST_STARTING_Y_POSITION, Rotation2d.fromDegrees(FIRST_STARTING_HEADING_ANGLE)),
            END_POSE_FOR_GOTO_SECOND_INTAKE_POSE = new Pose2d(SECOND_STARTING_X_POSITION, SECOND_STARTING_Y_POSITION, Rotation2d.fromDegrees(SECOND_STARTING_HEADING_ANGLE)),
            END_LEFT_POSE_FOR_GOTO_FIRST_INTAKE_POSE = FieldConstants.mirror(END_POSE_FOR_GOTO_FIRST_INTAKE_POSE, Rotation2d.fromDegrees(LEFT_FIRST_STARTING_HEADING_ANGLE)),
            END_LEFT_POSE_FOR_GOTO_SECOND_INTAKE_POSE = FieldConstants.mirror(END_POSE_FOR_GOTO_SECOND_INTAKE_POSE, Rotation2d.fromDegrees(LEFT_SECOND_STARTING_HEADING_ANGLE)),

            RIGHT_TRENCH_ENTRANCE_POSE = new Pose2d(ENTER_RIGHT_TRENCH_X_POSITION, ENTER_RIGHT_TRENCH_Y_POSITION, Rotation2d.fromDegrees(HEADING_ANGLE_FROM_ALLIANCE_ZONE)),
            RIGHT_TRENCH_EXIT_POSE = new Pose2d(EXIT_RIGHT_TRENCH_X_POSITION, EXIT_RIGHT_TRENCH_Y_POSITION, Rotation2d.fromDegrees(HEADING_ANGLE_FROM_ALLIANCE_ZONE)),

            LEFT_TRENCH_ENTRANCE_POSE = new Pose2d(ENTER_LEFT_TRENCH_X_POSITION, ENTER_LEFT_TRENCH_Y_POSITION, Rotation2d.fromDegrees(HEADING_ANGLE_TO_ALLIANCE_ZONE)),
            LEFT_TRENCH_EXIT_POSE = new Pose2d(EXIT_LEFT_TRENCH_X_POSITION, EXIT_LEFT_TRENCH_Y_POSITION, Rotation2d.fromDegrees(HEADING_ANGLE_TO_ALLIANCE_ZONE));


    public static final double
            AUTONOMOUS_TIME_SECONDS = 20,
            SCORING_TIMEOUT_SECONDS = 5,
            NORMAL_DELIVERY_TIMEOUT = 4,
            NORMAL_INTAKE_TIMEOUT = 3,
            NORMAL_DRIVE_TIMEOUT = 4;

    public static final PathConstraints
            DRIVE_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(3, 3.3, Units.degreesToRadians(500), Units.degreesToRadians(900)),
            SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_CONSTRAINTS = new PathConstraints(0.2, 0.3, Units.degreesToRadians(100), Units.degreesToRadians(100)),
            DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS = new PathConstraints(1.5, 1, Units.degreesToRadians(100), Units.degreesToRadians(100));

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(0, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(0, 0, 0) :
                    new PIDConstants(0, 0, 0);

    private static final PPHolonomicDriveController AUTO_PATH_FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS
    );

    /**
     * Initializes PathPlanner. This needs to be called before any PathPlanner function can be used.
     */
    public static void init() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        configureAutoBuilder();
        registerCommands();
        AutonomousGenerator.init();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose,
                RobotContainer.ROBOT_POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeChassisSpeeds,
                RobotContainer.SWERVE::drivePathPlanner,
                AUTO_PATH_FOLLOWING_CONTROLLER,
                ROBOT_CONFIG,
                Flippable::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("IntakeCommand", IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).withTimeout(1));
        NamedCommands.registerCommand("ShootCommand", ShootingCommands.getAutonomousShootingAtHubCommand().withTimeout(3));
    }
}