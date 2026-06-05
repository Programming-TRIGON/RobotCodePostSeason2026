package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Random;

public class SimulatedGamePiece {
    private static final ArrayList<SimulatedGamePiece> SIMULATED_GAME_PIECES = new ArrayList<>();

    // Represents occupied cells in the 3D hopper volume (depth, width, height).
    private static final ArrayList<HopperCell> OCCUPIED_HOPPER_CELLS = new ArrayList<>();

    private Translation3d fieldRelativePosition;
    private boolean isIndexed = true;
    private HopperCell hopperCell; // null when not held

    public SimulatedGamePiece(double startingPoseXMeters, double startingPoseYMeters) {
        SimulatedGamePieceConstants.GamePieceType gamePieceType = SimulatedGamePieceConstants.GamePieceType.FUEL;
        fieldRelativePosition = new Translation3d(startingPoseXMeters, startingPoseYMeters, gamePieceType.originPointHeightOffGroundMeters);
        SIMULATED_GAME_PIECES.add(this);
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedGamePieces() {
        return SIMULATED_GAME_PIECES;
    }

    public static ArrayList<SimulatedGamePiece> getUnheldGamePieces() {
        final ArrayList<SimulatedGamePiece> unheldGamePieces = new ArrayList<>(SIMULATED_GAME_PIECES);
        unheldGamePieces.removeIf(SimulatedGamePiece::isHeld);
        return unheldGamePieces;
    }

    public void updatePosition(Translation3d fieldRelativePosition) {
        this.fieldRelativePosition = fieldRelativePosition;
    }

    public Translation3d getPosition() {
        return fieldRelativePosition;
    }

    public boolean isScoredInHub() {
        return getPosition().getDistance(SimulatedGamePieceConstants.SCORE_CHECK_POSITION.get()) < SimulatedGamePieceConstants.SCORE_TOLERANCE_METERS;
    }

    void release() {
        if (hopperCell != null)
            OCCUPIED_HOPPER_CELLS.remove(hopperCell);
        hopperCell = null;
        isIndexed = false;
    }

    double getDistanceFromPositionMeters(Translation3d position) {
        return fieldRelativePosition.getDistance(position);
    }

    boolean isIndexed() {
        return isIndexed;
    }

    void resetIndexing() {
        hopperCell = calculateNextAvailableHopperCell();
        if (hopperCell != null) {
            isIndexed = true;
            OCCUPIED_HOPPER_CELLS.add(hopperCell);
        } else
            isIndexed = false;
    }

    HopperCell getHopperCell() {
        return hopperCell;
    }

    static void logAll() {
        Logger.recordOutput("Poses/GamePieces/Fuel", getSimulatedFuelAsPoseArray());
    }

    private static Pose3d[] getSimulatedFuelAsPoseArray() {
        final Pose3d[] poses = new Pose3d[SimulatedGamePiece.SIMULATED_GAME_PIECES.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = new Pose3d(SimulatedGamePiece.SIMULATED_GAME_PIECES.get(i).getPosition(), new Rotation3d());
        return poses;
    }

    private boolean isHeld() {
        return hopperCell != null;
    }

    /**
     * Finds the next open cell in the hopper, filling it like a real bin:
     * <p>
     * - The floor layer (layer 0) is filled front-to-back. Each row only accepts a randomized
     * number of balls (between {@link SimulatedGamePieceConstants#MINIMUM_BALLS_PER_ROW} and the
     * full width), so rows look uneven instead of a perfect 4-wide grid.
     * <p>
     * - The front rows (depth index below {@link SimulatedGamePieceConstants#NON_STACKING_ROW_COUNT})
     * are the flat shooter-feed section and never receive a second layer. Only the rear rows, which
     * sit on the indexer ramp, are allowed to stack upward.
     */
    private static HopperCell calculateNextAvailableHopperCell() {
        final int depth = SimulatedGamePieceConstants.HOPPER_DEPTH_CAPACITY;
        final int maxLayers = SimulatedGamePieceConstants.HOPPER_HEIGHT_CAPACITY;

        for (int layer = 0; layer < maxLayers; layer++) {
            for (int depthIndex = 0; depthIndex < depth; depthIndex++) {
                if (!canRowHoldLayer(depthIndex, layer))
                    continue;

                final HopperCell freeCell = findFreeCellInRow(depthIndex, layer);
                if (freeCell != null)
                    return freeCell;
            }
        }

        return null;
    }

    /**
     * The flat front rows (shooter feed) hold a single layer only. Rear rows on the ramp may stack.
     */
    private static boolean canRowHoldLayer(int depthIndex, int layer) {
        if (layer == 0)
            return true;
        return depthIndex >= SimulatedGamePieceConstants.NON_STACKING_ROW_COUNT;
    }

    /**
     * Returns the first open cell within a row's randomized, centered set of occupied columns,
     * or null if that row+layer is already full.
     */
    private static HopperCell findFreeCellInRow(int depthIndex, int layer) {
        final int ballsInRow = calculateRowCapacity(depthIndex, layer);
        final int width = SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY;

        // Center the occupied columns within the full width so partial rows sit in the middle.
        final int startColumn = (width - ballsInRow) / 2;

        for (int offset = 0; offset < ballsInRow; offset++) {
            final int widthIndex = startColumn + offset;
            final HopperCell candidate = new HopperCell(depthIndex, widthIndex, layer);
            if (!OCCUPIED_HOPPER_CELLS.contains(candidate))
                return candidate;
        }

        return null;
    }

    /**
     * Deterministically picks how many balls a given row holds, so the count is stable frame to
     * frame but varies from row to row (e.g. sometimes only 2 or 3 across instead of the full 4).
     */
    private static int calculateRowCapacity(int depthIndex, int layer) {
        final int width = SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY;
        final int minimum = SimulatedGamePieceConstants.MINIMUM_BALLS_PER_ROW;

        final Random rowRNG = new Random(depthIndex * 92821L + layer * 53L + SimulatedGamePieceConstants.ROW_CAPACITY_SEED);
        return minimum + rowRNG.nextInt(width - minimum + 1);
    }

    /**
     * A discrete cell inside the hopper grid.
     * depthIndex runs front (0) to back along the robot's X axis,
     * widthIndex runs across the robot's Y axis,
     * layerIndex runs bottom (0) to top along Z.
     */
    record HopperCell(int depthIndex, int widthIndex, int layerIndex) {
    }
}