package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.LocalADStarAK;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.AutonomousCommands;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * A class that contains the constants and configurations for everything related to the 15-second autonomous period at the start of the match.
 */
public class AutonomousConstants {
    public static final String DEFAULT_AUTO_NAME = "DefaultAutoName";
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = RobotHardwareStats.isSimulation()
            ? 0.3
            : 0.5;//TODO: Calibrate

    public static final double
            TOTAL_MATCH_TIME_SECONDS = 160,
            AUTONOMOUS_TIME_SECONDS = 20,
            DEPOT_COLLECTION_TIMEOUT_SECONDS = 4,
            NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS = 2,
            SCORING_TIMEOUT_SECONDS = 3,
            NORMAL_DRIVE_TIMEOUT = 4,
            AUTONOMOUS_SHOOTING_DURATION_SECONDS = 5,
            AUTONOMOUS_DELIVERY_DURATION_SECONDS = 7;
    public static final Rotation2d DELIVERY_ROTATION = Rotation2d.fromDegrees(27.873);

    public static boolean IS_AUTO_LEFT_SIDE = true;
    public static final FlippablePose2d
            DOUBLE_SWIPE_LEFT_SHOOTING_POSE = new FlippablePose2d(
            2.853, 6.768,
            Rotation2d.fromDegrees(120.390),
            true
    ),
            DOUBLE_SWIPE_RIGHT_SHOOTING_POSE = new FlippablePose2d(
                    DOUBLE_SWIPE_LEFT_SHOOTING_POSE.get().getX(), FieldConstants.FIELD_WIDTH_METERS - DOUBLE_SWIPE_LEFT_SHOOTING_POSE.get().getY(),
                    DOUBLE_SWIPE_LEFT_SHOOTING_POSE.get().getRotation().unaryMinus(),
                    true
            ),
            NEUTRAL_INTAKE_POSE = new FlippablePose2d(
                    7.360, 7.003,
                    Rotation2d.fromDegrees(-31.215),
                    true
            );


    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(0, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(0, 0, 0) :
                    new PIDConstants(0, 0, 0);


    public static final PIDController GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    public static final ProfiledPIDController GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    public static final double AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS = 2;

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
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose,
                (a) -> {
                },
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
        NamedCommands.registerCommand("CollectCommand", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN));
        NamedCommands.registerCommand("ShootCommand", AutonomousCommands.getTimedScoreCommand(AUTONOMOUS_SHOOTING_DURATION_SECONDS));
        NamedCommands.registerCommand("PrepareForShootCommand", ShootingCommands.getPrepareForFixedAutonomousShootingCommand());
    }
}