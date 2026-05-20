package frc.trigon.robot.misc.shootingcalculations.shootingvisualization;

import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

public class FuelShootingVisualizationConstants {
    static final double G_FORCE = 9.794; // Gravity

    static final double
            GAME_PIECE_MASS_KG = 0.21,
            GAME_PIECE_RADIUS_METERS = 0.075,
            GAME_PIECE_AREA = Math.PI * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS;

    // Adjusted drag for symmetric drum shot
    static final double
            AIR_DENSITY = 1.225,
            DRAG_COEFFICIENT = 0.55;

    static final double SIMULATION_TIME_STEP_SECONDS = 0.001;
    static final double END_SIMULATION_HEIGHT_METERS = SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters;
}