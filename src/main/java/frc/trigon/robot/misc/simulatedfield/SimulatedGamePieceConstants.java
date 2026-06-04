package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation3d;
import frc.trigon.robot.constants.FieldConstants;

public class SimulatedGamePieceConstants {
    public static final double SCORE_TOLERANCE_METERS = 0.3;
    static final double INTAKE_TOLERANCE_METERS = 0.4;

    public static final FlippableTranslation3d
            SCORE_CHECK_POSITION = new FlippableTranslation3d(new Translation3d(4.625594, FieldConstants.FIELD_WIDTH_METERS / 2, 1.4), true),
            EJECT_FUEL_FROM_HUB_POSITION = new FlippableTranslation3d(new Translation3d(5.189474, FieldConstants.FIELD_WIDTH_METERS / 2, 0.762), true);

    static final Translation3d COLLECTION_CHECK_POSITION = new Translation3d(0.4, 0, 0); //TODO: get
    static final int MAXIMUM_HELD_FUEL = 40; //TODO: get

    // Indexer width: still used by ShootingCalculations to lay out the per-column shooter exit lanes.
    public static final int INDEXER_WIDTH_CAPACITY = 4;
    public static final double INDEXER_COL_SPACING_METERS = 0.16;

    // 3D hopper volume definition. Fuel fills the bottom layer first (depth x width),
    // then stacks upward layer by layer, keeping the pile inside the hopper bounds.
    // depth * width * height should be >= MAXIMUM_HELD_FUEL so every held piece gets a cell.
    public static final int
            HOPPER_DEPTH_CAPACITY = 4,   // cells along robot X (front-to-back)
            HOPPER_WIDTH_CAPACITY = 4,   // cells along robot Y (left-to-right)
            HOPPER_HEIGHT_CAPACITY = 4;  // stacked layers along Z

    // Physical spacing between cell centers inside the hopper.
    public static final double
            HOPPER_DEPTH_SPACING_METERS = 0.15,
            HOPPER_WIDTH_SPACING_METERS = 0.15,
            HOPPER_LAYER_SPACING_METERS = 0.13; // < ball diameter so layers nestle slightly

    // Visual jitter so the pile reads as loose balls rather than a perfect lattice.
    public static final double ORGANIC_SCATTER_METERS = 0.02;

    public static final double FUEL_DIAMETER_METERS = 0.15;

    private static final int
            STARTING_FUEL_ROWS = 12,
            STARTING_FUEL_COLUMNS = 30;

    private static final double
            STARTING_FUEL_X_POSITION_METERS = 7.357364,
            STARTING_FUEL_Y_POSITION_METERS = 1.724406,
            STARTING_FUEL_SPACING_METERS = 0.16;

    private static final int
            DEPOT_FUEL_ROWS = 6,
            DEPOT_FUEL_COLUMNS = 4;
    private static final Translation2d DEPOT_CENTER_POSITION = new Translation2d(0.31, 5.96);

    public static final double
            EJECTION_FROM_HUB_MINIMUM_VELOCITY_METERS_PER_SECOND = 4,
            EJECTION_FROM_HUB_MAXIMUM_VELOCITY_METERS_PER_SECOND = 15;
    public static final Rotation2d EJECTION_FROM_HUB_MAXIMUM_ANGLE = Rotation2d.fromDegrees(35);

    static {
        initializeFuel();
    }

    private static void initializeFuel() {
        for (int i = 0; i < STARTING_FUEL_ROWS; i++) {
            for (int j = 0; j < STARTING_FUEL_COLUMNS; j++) {
                new SimulatedGamePiece(
                        STARTING_FUEL_X_POSITION_METERS + (i * STARTING_FUEL_SPACING_METERS),
                        STARTING_FUEL_Y_POSITION_METERS + (j * STARTING_FUEL_SPACING_METERS)
                );
            }
        }

        // Initialize pre-loaded fuel in the hopper.
        for (int i = 0; i < 8; i++) {
            final SimulatedGamePiece currentHeldFuel = new SimulatedGamePiece(0, 0);
            SimulationFieldHandler.addHeldFuel(currentHeldFuel);
        }

        initializeDepotFuel(DEPOT_CENTER_POSITION.getX(), DEPOT_CENTER_POSITION.getY());
        initializeDepotFuel(FieldConstants.FIELD_LENGTH_METERS - DEPOT_CENTER_POSITION.getX(), FieldConstants.FIELD_WIDTH_METERS - DEPOT_CENTER_POSITION.getY());
    }

    private static void initializeDepotFuel(double depotCenterX, double depotCenterY) {
        double startX = depotCenterX - (DEPOT_FUEL_COLUMNS - 1) * STARTING_FUEL_SPACING_METERS / 2.0;
        double startY = depotCenterY - (DEPOT_FUEL_ROWS - 1) * STARTING_FUEL_SPACING_METERS / 2.0;

        for (int row = 0; row < DEPOT_FUEL_ROWS; row++) {
            for (int col = 0; col < DEPOT_FUEL_COLUMNS; col++) {
                new SimulatedGamePiece(
                        startX + col * STARTING_FUEL_SPACING_METERS,
                        startY + row * STARTING_FUEL_SPACING_METERS
                );
            }
        }
    }

    public enum GamePieceType {
        FUEL(FUEL_DIAMETER_METERS / 2.0, 0);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }
    }
}