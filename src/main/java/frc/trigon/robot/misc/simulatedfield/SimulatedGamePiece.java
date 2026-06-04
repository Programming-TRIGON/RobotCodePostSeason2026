package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

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
     * Finds the next open cell in the hopper, filling it like a real bin: the floor layer is
     * filled first (front-to-back across the depth, then across the width), and only once a
     * layer is full does fuel begin stacking on the layer above it. This keeps the pile inside
     * the hopper volume instead of climbing diagonally up one side.
     */
    private static HopperCell calculateNextAvailableHopperCell() {
        final int depth = SimulatedGamePieceConstants.HOPPER_DEPTH_CAPACITY;
        final int width = SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY;
        final int maxLayers = SimulatedGamePieceConstants.HOPPER_HEIGHT_CAPACITY;

        for (int layer = 0; layer < maxLayers; layer++) {
            for (int depthIndex = 0; depthIndex < depth; depthIndex++) {
                for (int widthIndex = 0; widthIndex < width; widthIndex++) {
                    final HopperCell candidate = new HopperCell(depthIndex, widthIndex, layer);
                    if (!OCCUPIED_HOPPER_CELLS.contains(candidate))
                        return candidate;
                }
            }
        }

        return null;
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