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
        // Held membership is owned by SimulationFieldHandler's HELD_FUEL list, NOT by whether a
        // hopper cell happens to be allocated. A held piece that couldn't get a cell (full hopper)
        // must still count as held, or getUnheldGamePieces() would expose it as loose fuel and it
        // could be collected twice.
        return SimulationFieldHandler.isHeld(this);
    }

    /**
     * Finds the next open cell in the hopper, filling it like a real bin:
     * <p>
     * - The floor layer (layer 0) is filled front-to-back. Each row only accepts a randomized
     * number of balls (between {@link SimulatedGamePieceConstants#MINIMUM_BALLS_PER_ROW} and the
     * full width), so rows look uneven instead of a perfect 4-wide grid.
     * <p>
     * - The very front row (depth 0) is the flat shooter-feed section and stays a single layer.
     * The second row stacks to a reduced height, and the rear rows stack to full height, so the
     * pile tapers down toward the shooter.
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
     * Per-row stacking limit. The very front row (shooter feed) is a single flat layer. The
     * second row may stack, but only to a reduced height so it tapers down toward the shooter.
     * The remaining rear rows stack to the full hopper height.
     */
    private static boolean canRowHoldLayer(int depthIndex, int layer) {
        if (layer == 0)
            return true;
        if (depthIndex < SimulatedGamePieceConstants.NON_STACKING_ROW_COUNT)
            return false;
        if (depthIndex == SimulatedGamePieceConstants.NON_STACKING_ROW_COUNT)
            return layer < SimulatedGamePieceConstants.SECOND_ROW_MAX_LAYERS;
        return true;
    }

    /**
     * Returns the first open cell within a row's randomized, centered set of occupied columns,
     * or null if that row+layer is already full.
     */
    private static HopperCell findFreeCellInRow(int depthIndex, int layer) {
        final int ballsInRow = calculateRowCapacity(depthIndex);
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
     * Deterministically picks how many balls a given depth row holds, so the count is stable
     * frame to frame but varies from row to row (e.g. sometimes only 2 or 3 across instead of the
     * full 4). The width is keyed on depthIndex ONLY, never the layer, so an upper layer can never
     * be wider than the layer beneath it (which would leave balls unsupported).
     */
    private static int calculateRowCapacity(int depthIndex) {
        final int width = SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY;
        final int minimum = SimulatedGamePieceConstants.MINIMUM_BALLS_PER_ROW;

        final Random rowRNG = new Random(depthIndex * 92821L + SimulatedGamePieceConstants.ROW_CAPACITY_SEED);
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