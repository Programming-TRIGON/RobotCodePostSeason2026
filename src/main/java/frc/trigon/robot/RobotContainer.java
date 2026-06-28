// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.FieldRelativeRestrictedDriveCommand;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.ZoneRestrictionsDrive;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.RestrictedZone;
import frc.trigon.robot.commands.commandfactories.*;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
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
//        configureSysIDBindings(SHOOTER);
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
//        configureSysIDBindings(SHOOTER);
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        HOOD.setDefaultCommand(HoodCommands.getRestCommand());
        INDEXER.setDefaultCommand(IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.AGITATE));
        INTAKE.setDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPEN));
        LOADER.setDefaultCommand(LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.REST));
        SHOOTER.setDefaultCommand(ShooterCommands.getStopCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());
        OperatorConstants.CAMERAS_DISCONNECTED_TRIGGER.onTrue(CommandConstants.INDICATE_CAMERAS_DISCONNECTED_COMMAND);
        OperatorConstants.DEBUGGING_TRIGGER.whileTrue(GeneralCommands.getDebuggingCommand());
        OperatorConstants.SHOOTING_MAP_CALIBRATION_TRIGGER.whileTrue(ShootingCommands.getShootingMapCalibrationCommand());
        OperatorConstants.TELEPORTATION_FOR_SIMULATION_SHOOTING_MAP_CALIBRATION_TRIGGER.whileTrue(GeneralCommands.getTeleportRobotForSimulationShootingMapCalibrationCommand(ShootingCalculations.TargetShootingLocation.RIGHT_DELIVERY_LOCATION));

        OperatorConstants.SHOOTING_TRIGGER.whileTrue(ShootingCommands.getShootingCommand());
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_HUB_POSITION_IN_FRONT_OF_TOWER_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.IN_FRONT_OF_TOWER));
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_HUB_POSITION_RIGHT_TRENCH_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.RIGHT_TRENCH));
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_HUB_POSITION_LEFT_TRENCH_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.LEFT_TRENCH));
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_HUB_POSITION_BACK_RIGHT_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.BACK_RIGHT));
        OperatorConstants.SET_TARGET_FIXED_SCORING_IN_HUB_POSITION_BACK_LEFT_TRIGGER.onTrue(ShootingCommands.getSetFixedShootingStateCommand(ShootingCommands.FixedShootingPosition.BACK_LEFT));
        OperatorConstants.FIXED_SHOOTING_AT_HUB_TRIGGER.whileTrue(ShootingCommands.getFixedShootingAtHubCommand());
        OperatorConstants.FIXED_DELIVERY_TRIGGER.whileTrue(ShootingCommands.getFixedDeliveryShootingCommand());
        OperatorConstants.PREPARE_FOR_SHOOTING_TRIGGER.whileTrue(ShootingCommands.getPrepareForShootingCommand());
        OperatorConstants.EJECT_FROM_INTAKE_TRIGGER.whileTrue(EjectionCommands.getEjectFromIntakeCommand());
        OperatorConstants.EJECT_FROM_SHOOTER_TRIGGER.whileTrue(EjectionCommands.getEjectFromShooterCommand());
        OperatorConstants.INTAKE_TRIGGER.and(OperatorConstants.SHOOTING_TRIGGER.negate()).whileTrue(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.POWERED_OPEN)).and(CommandConstants::shouldUseIntakeAssist).whileTrue(CommandConstants.INTAKE_CENTER_OF_ROTATION_COMMAND);
        OperatorConstants.PRELOAD_TRIGGER.onTrue(FuelIntakeCommands.getPreloadCommand());
        OperatorConstants.CLOSE_INTAKE_WITHOUT_SHOOTING_TRIGGER.whileTrue(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.CLOSE));
        OperatorConstants.RESTRICT_HOOD_ANGLE_TRIGGER.whileTrue(HoodCommands.getRestCommand());
        OperatorConstants.TRENCH_ASSIST_TRIGGER.whileTrue(CommandConstants.TRENCH_ASSIST_COMMAND);
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
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}