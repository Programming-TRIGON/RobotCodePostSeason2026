// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.*;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.matchTracker.MatchTrackerCommands;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.poseestimation.robotposeestimator.RobotPoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.hood.Hood;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.indexer.Indexer;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.Loader;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class RobotContainer {
    public static final RobotPoseEstimator ROBOT_POSE_ESTIMATOR = new RobotPoseEstimator(
            CameraConstants.RIGHT_APRIL_TAG_CAMERA,
            CameraConstants.LEFT_APRIL_TAG_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final Hood HOOD = new Hood();
    public static final Indexer INDEXER = new Indexer();
    public static final Intake INTAKE = new Intake();
    public static final Loader LOADER = new Loader();
    public static final Shooter SHOOTER = new Shooter();
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();

//        configureSysIDBindings(INTAKE);
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        AutonomousCommands.IS_AUTO_LEFT_SIDE = !autoChooser.get().getName().endsWith("Right");
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        HOOD.setDefaultCommand(HoodCommands.getRestCommand());
        INDEXER.setDefaultCommand(IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.REST));
        INTAKE.setDefaultCommand(IntakeCommands.getDefaultCommand());
        LOADER.setDefaultCommand(LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.REST));
        SHOOTER.setDefaultCommand(ShooterCommands.getStopCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());
        OperatorConstants.DEBUGGING_TRIGGER.whileTrue(GeneralCommands.getDebuggingCommand());
        OperatorConstants.SHOOTING_MAP_CALIBRATION_TRIGGER.whileTrue(ShootingCommands.getShootingMapCalibrationCommand());
        OperatorConstants.TELEPORTATION_FOR_SIMULATION_SHOOTING_MAP_CALIBRATION_TRIGGER.whileTrue(GeneralCommands.getTeleportRobotForSimulationShootingMapCalibrationCommand(ShootingCalculations.TargetShootingLocation.RIGHT_DELIVERY_LOCATION));
        OperatorConstants.RESET_POSE_TO_FIXED_SHOOTING_LOCATION_TRIGGER.onTrue(ShootingCommands.getResetPoseToFixedShootingLocationCommand());
        OperatorConstants.RESET_HOOD_TRIGGER.whileTrue(HoodCommands.getResetHoodCommand());

        OperatorConstants.SHOOTING_TRIGGER.whileTrue(ShootingCommands.getShootingCommand());
        OperatorConstants.SET_TARGET_FIXED_SCORING_BETWEEN_TOWER_AND_HUB_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.BETWEEN_TOWER_AND_HUB));
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_FRONT_OF_TOWER_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.IN_FRONT_OF_TOWER));
        OperatorConstants.SET_TARGET_FIXED_SCORING_CLOSEST_TRENCH_TRIGGER.onTrue(ShootingCommands.getSetClosestTrenchCommand());
        OperatorConstants.SET_TARGET_FIXED_SCORING_CLOSEST_SIDE_OF_TOWER_TRIGGER.onTrue(ShootingCommands.getSetClosestSideOfTowerCommand());
        OperatorConstants.SET_TARGET_FIXED_SCORING_RIGHT_TRENCH_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.RIGHT_TRENCH));
        OperatorConstants.SET_TARGET_FIXED_SCORING_LEFT_TRENCH_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.LEFT_TRENCH));
        OperatorConstants.SET_TARGET_FIXED_SCORING_RIGHT_OF_TOWER_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.RIGHT_OF_TOWER));
        OperatorConstants.SET_TARGET_FIXED_SCORING_LEFT_OF_TOWER_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.LEFT_OF_TOWER));
        OperatorConstants.SET_TARGET_FIXED_SCORING_BACK_LEFT_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.BACK_LEFT));
        OperatorConstants.SET_TARGET_FIXED_SCORING_BACK_RIGHT_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.BACK_RIGHT));

        OperatorConstants.FIXED_SHOOTING_AT_HUB_TRIGGER.whileTrue(ShootingCommands.getFixedShootingAtHubCommand());
        OperatorConstants.PREPARE_FOR_FIXED_SHOOTING_TRIGGER.whileTrue(ShootingCommands.getPrepareForFixedShootingCommand());
        OperatorConstants.FIXED_DELIVERY_TRIGGER.whileTrue(ShootingCommands.getFixedDeliveryShootingCommand());
        OperatorConstants.PREPARE_FOR_SHOOTING_TRIGGER.whileTrue(ShootingCommands.getPrepareForShootingCommand());

        OperatorConstants.EJECT_FROM_INTAKE_TRIGGER.whileTrue(EjectionCommands.getEjectFromIntakeCommand());
        OperatorConstants.EJECT_FROM_SHOOTER_TRIGGER.whileTrue(EjectionCommands.getEjectFromShooterCommand());
        OperatorConstants.ENABLE_OVERRIDE_FIXED_SWERVE_AIM_TRIGGER.onTrue(ShootingCommands.getEnableFixedOverrideSwerveAimCommand());
        OperatorConstants.DISABLE_OVERRIDE_FIXED_SWERVE_AIM_TRIGGER.onTrue(ShootingCommands.getDisableFixedOverrideSwerveAimCommand());
        OperatorConstants.INTAKE_TRIGGER.and(OperatorConstants.SHOOTING_TRIGGER.negate()).whileTrue(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)).and(CommandConstants::shouldUseIntakeAssist).whileTrue(CommandConstants.INTAKE_CENTER_OF_ROTATION_COMMAND);
        OperatorConstants.PRELOAD_TRIGGER.onTrue(FuelIntakeCommands.getPreloadCommand());
        OperatorConstants.CLOSE_INTAKE_WITHOUT_SHOOTING_TRIGGER.whileTrue(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE));
        OperatorConstants.TRENCH_ASSIST_TRIGGER.whileTrue(CommandConstants.TRENCH_ASSIST_COMMAND);
        OperatorConstants.HUB_ACTIVE_STATE_CHANGED_TRIGGER.onTrue(MatchTrackerCommands.getRumbleCommand());
        OperatorConstants.OPEN_INTAKE_DEFAULT_COMMAND.onTrue(new InstantCommand(() -> FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.set(true)));
        OperatorConstants.CLOSE_INTAKE_DEFAULT_COMMAND.onTrue(new InstantCommand(() -> FuelIntakeCommands.SHOULD_INTAKE_DEFAULT_OPEN.set(false)));
    }

    private void configureSysIDBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    /**
     * Initializes the general systems of the robot.
     * Some systems need to be initialized at the start of the robot code so that others can use their functions.
     * For example, the LEDConstants need to be initialized so that the other systems can use them.
     */
    private void initializeGeneralSystems() {
        Flippable.init();
        LEDConstants.init();
        AutonomousConstants.init();
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser");

        final List<String> autoNames = AutoBuilder.getAllAutoNames();
        boolean hasDefault = false;

        for (String autoName : autoNames) {
            final Command autoNonMirrored = Commands.runOnce(() -> AutonomousCommands.IS_AUTO_LEFT_SIDE = true).andThen(new PathPlannerAuto(autoName));
            final Command autoMirrored = Commands.runOnce(() -> AutonomousCommands.IS_AUTO_LEFT_SIDE = false).andThen(new PathPlannerAuto(autoName, true));
            final String leftName = autoName + " Left";
            final String rightName = autoName + " Right";

            if (!AutonomousConstants.DEFAULT_AUTO_NAME.isEmpty() && AutonomousConstants.DEFAULT_AUTO_NAME.equals(autoName)) {
                hasDefault = true;
                autoChooser.addDefaultOption(leftName, autoNonMirrored);
                autoChooser.addOption(rightName, autoMirrored);
            } else if (!AutonomousConstants.DEFAULT_AUTO_NAME.isEmpty() && AutonomousConstants.DEFAULT_AUTO_NAME.equals(autoName + "Mirrored")) {
                hasDefault = true;
                autoChooser.addDefaultOption(rightName, autoMirrored);
                autoChooser.addOption(leftName, autoNonMirrored);
            } else {
                autoChooser.addOption(leftName, autoNonMirrored);
                autoChooser.addOption(rightName, autoMirrored);
            }
        }

        if (!hasDefault)
            autoChooser.addDefaultOption("None", Commands.none());
        else
            autoChooser.addOption("None", Commands.none());
    }
}