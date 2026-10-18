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
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.commands.commandfactories.autonomous.AutonomousGenerator;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.json.simple.parser.ParseException;

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

    private final static double
            END_PATH_VELOCITY = 0.0,
            FIRST_STARTING_X_POSITION = 8.25,
            FIRST_STARTING_Y_POSITION = 1.3,
            FIRST_END_X_POSITION = 8.25,
            FIRST_END_Y_POSITION = 4.0,
            FIRST_STARTING_HEADING_ANGLE = 90.0,
            FIRST_END_HEADING_ANGLE = -90.0,
            SECOND_STARTING_X_POSITION = 7.0,
            SECOND_STARTING_Y_POSITION = 1.3,
            SECOND_END_X_POSITION = 7.0,
            SECOND_END_Y_POSITION = 4.0,
            SECOND_STARTING_HEADING_ANGLE = 90.0,
            SECOND_END_HEADING_ANGLE = -90.0;

    private final static double
            INTAKE_MAX_VELOCITY_MPS = 3.0,
            INTAKE_MAX_ACCELERATION_MPSSQ = 3.0,
            INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC = 2 * Math.PI,
            INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = 4 * Math.PI;

    private final static List<Waypoint> FIRST_INTAKE_WAYPOINT = PathPlannerPath.waypointsFromPoses(
            new Pose2d(FIRST_STARTING_X_POSITION, FIRST_STARTING_Y_POSITION, Rotation2d.fromDegrees(FIRST_STARTING_HEADING_ANGLE)),
            new Pose2d(FIRST_END_X_POSITION, FIRST_END_Y_POSITION, Rotation2d.fromDegrees(FIRST_END_HEADING_ANGLE))
    );

    private final static List<Waypoint> SECOND_INTAKE_WAYPOINT =PathPlannerPath.waypointsFromPoses(
            new Pose2d(SECOND_STARTING_X_POSITION, SECOND_STARTING_Y_POSITION, Rotation2d.fromDegrees(SECOND_STARTING_HEADING_ANGLE)),
            new Pose2d(SECOND_END_X_POSITION, SECOND_END_Y_POSITION, Rotation2d.fromDegrees(SECOND_END_HEADING_ANGLE))
    );

    private final static PathConstraints INTAKE_CONSTRAINTS = new PathConstraints(
            INTAKE_MAX_VELOCITY_MPS,
            INTAKE_MAX_ACCELERATION_MPSSQ,
            INTAKE_MAX_ANGULAR_VELOCITY_PER_SEC,
            INTAKE_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ
    );

    public static PathPlannerPath FIRST_INTAKE_PATH = new PathPlannerPath(
            FIRST_INTAKE_WAYPOINT,
            INTAKE_CONSTRAINTS,
            null,
            new GoalEndState(END_PATH_VELOCITY, Rotation2d.fromDegrees(FIRST_END_HEADING_ANGLE))
    );

    public static PathPlannerPath SECOND_INTAKE_PATH = new PathPlannerPath(
            SECOND_INTAKE_WAYPOINT,
            INTAKE_CONSTRAINTS,
            null,
            new GoalEndState(END_PATH_VELOCITY, Rotation2d.fromDegrees(FIRST_END_HEADING_ANGLE))
    );

    public static final double
            AUTONOMOUS_TIME_SECONDS = 20,
            SCORING_TIMEOUT_SECONDS = 5,
            NORMAL_DELIVERY_TIMEOUT = 4,
            NORMAL_DRIVE_TIMEOUT = 4;

    public static final double
            AUTONOMOUS_SHOOTING_DURATION_SECONDS = 5,
            AUTONOMOUS_DELIVERY_DURATION_SECONDS = 7;

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