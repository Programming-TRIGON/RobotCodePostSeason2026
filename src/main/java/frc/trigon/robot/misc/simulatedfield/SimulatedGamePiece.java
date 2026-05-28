package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class SimulatedGamePiece {
    private static final ArrayList<SimulatedGamePiece> SIMULATED_GAME_PIECES = new ArrayList<>();

    // Represents occupied slots [row, col] in the 4-wide indexer
    private static final ArrayList<Translation2d> OCCUPIED_INDEXER_SLOTS = new ArrayList<>();

    private Translation3d fieldRelativePosition;
    private boolean isIndexed = true;
    private Translation2d indexerGridSlot; // X = row, Y = column

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
        if (indexerGridSlot != null) {
            OCCUPIED_INDEXER_SLOTS.remove(indexerGridSlot);
        }
        indexerGridSlot = null;
        isIndexed = false;
    }

    double getDistanceFromPositionMeters(Translation3d position) {
        return fieldRelativePosition.getDistance(position);
    }

    boolean isIndexed() {
        return isIndexed;
    }

    void resetIndexing() {
        indexerGridSlot = calculateNextAvailableIndexerSlot();
        if (indexerGridSlot != null) {
            isIndexed = true;
            OCCUPIED_INDEXER_SLOTS.add(indexerGridSlot);
        } else {
            isIndexed = false;
        }
    }

    Translation2d getIndexerGridSlot() {
        return indexerGridSlot;
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
        return indexerGridSlot != null;
    }

    /**
     * Finds the next open row and column in the 4-wide roller indexer queue.
     */
    private Translation2d calculateNextAvailableIndexerSlot() {
        int targetRow = 0;

        while (true) {
            for (int col = 0; col < SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY; col++) {
                Translation2d candidateSlot = new Translation2d(targetRow, col);
                if (!OCCUPIED_INDEXER_SLOTS.contains(candidateSlot)) {
                    return candidateSlot;
                }
            }
            targetRow++;
            // Failsafe to prevent infinite loops if loaded beyond physical reality
            if (targetRow > (SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL / SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY) + 1) {
                return null;
            }
        }
    }
}