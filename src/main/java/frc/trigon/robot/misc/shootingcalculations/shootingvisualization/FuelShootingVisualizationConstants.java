package frc.trigon.robot.misc.shootingcalculations.shootingvisualization;

import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

public class FuelShootingVisualizationConstants {
    static final double G_FORCE = 9.794;

    static final double
            GAME_PIECE_MASS_KG = 0.21,
            GAME_PIECE_RADIUS_METERS = 0.075,
            GAME_PIECE_AREA = Math.PI * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS,
            MOMENT_OF_INERTIA = 2.0 / 5.0 * GAME_PIECE_MASS_KG * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS;

    static final double
            TOP_ROLLER_GEAR_RATIO = 1.5,//ShooterConstants.TOP_ROLLER_GEAR_RATIO,
            BOTTOM_ROLLER_GEAR_RATIO = 2, //ShooterConstants.BOTTOM_ROLLER_GEAR_RATIO;
            TOP_ROLLER_RADIUS_METERS = 0.025, //ShooterConstants.TOP_ROLLER_RADIUS_METERS,
            BOTTOM_ROLLER_RADIUS_METERS = Units.inchesToMeters(2); //ShooterConstants.BOTTOM_ROLLER_RADIUS_METERS

    // Adjusted drag for symmetric drum shot
    static final double
            AIR_DENSITY = 1.225,
            DRAG_COEFFICIENT = 0.55,
            MAGNUS_LIFT_FACTOR = 0.25,
            SPIN_DECAY_COEFFICIENT = 0.01;

    static final double SIMULATION_TIME_STEP_SECONDS = 0.001;
    static final double END_SIMULATION_HEIGHT_METERS = SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters;
}