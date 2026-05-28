package frc.trigon.robot.misc.shootingcalculations.shootingvisualization;

import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class FuelShootingVisualizationConstants {
    static final double G_FORCE = 9.794;

    static final double
            GAME_PIECE_MASS_KG = 0.21,
            GAME_PIECE_RADIUS_METERS = 0.075,
            GAME_PIECE_AREA = Math.PI * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS,
            MOMENT_OF_INERTIA = 2.0 / 5.0 * GAME_PIECE_MASS_KG * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS;

    static final double
            TOP_ROLLER_GEAR_RATIO = ShooterConstants.TOP_WHEEL_GEAR_RATIO,
            BOTTOM_ROLLER_GEAR_RATIO = ShooterConstants.BOTTOM_WHEEL_GEAR_RATIO,
            TOP_ROLLER_RADIUS_METERS = ShooterConstants.TOP_WHEEL_DIAMETER / 2,
            BOTTOM_ROLLER_RADIUS_METERS = ShooterConstants.BOTTOM_WHEEL_DIAMETER / 2;

    // Adjusted drag for symmetric drum shot
    static final double
            AIR_DENSITY = 1.225,
            DRAG_COEFFICIENT = 0.55,
            MAGNUS_LIFT_FACTOR = 0.25,
            SPIN_DECAY_COEFFICIENT = 0.01;

    static final double SIMULATION_TIME_STEP_SECONDS = 0.001;
    static final double END_SIMULATION_HEIGHT_METERS = SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters;
}