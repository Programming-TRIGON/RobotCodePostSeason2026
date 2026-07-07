package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.LocalADStarAK;
import frc.trigon.lib.utilities.flippable.Flippable;
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
            AUTONOMOUS_SHOOTING_DURATION_SECONDS = 5,
            AUTONOMOUS_DELIVERY_DURATION_SECONDS = 7;

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
        NamedCommands.registerCommand("CollectCommand", IntakeCommands.getSafeSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN));
        NamedCommands.registerCommand("FirstCollectCommand", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.AUTONOMOUS_INTAKE));
        NamedCommands.registerCommand("DoubleSwipeShootCommand", AutonomousCommands.getTimedScoreCommand(AUTONOMOUS_SHOOTING_DURATION_SECONDS));
        NamedCommands.registerCommand("DoubleSwipePrepareForShootCommand", ShootingCommands.getPrepareForDoubleSwipeFixedAutonomousShootingCommand());
        NamedCommands.registerCommand("BasicShootCommand", ShootingCommands.getBasicFixedAutonomousShootingCommand());
        NamedCommands.registerCommand("PushFuelCommand", AutonomousCommands.getPushFuelWithIntakeCommand());
        NamedCommands.registerCommand("PrepareForShootingCommand", ShootingCommands.getPrepareForShootingCommand());
        NamedCommands.registerCommand("ShootCommand", ShootingCommands.getAutonomousShootingAtHubCommand().withTimeout(3));
        NamedCommands.registerCommand("InitializeDriveCommand", new InstantCommand(()-> RobotContainer.SWERVE.initializeDrive(true)));
        NamedCommands.registerCommand("WaitForIntakeToOpenCommand", IntakeCommands.getAutonomousSafeSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN).until(RobotContainer.INTAKE::atTargetState));
    }
}