package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingcalculations.shootingvisualization.VisualizeFuelShootingCommand;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

import java.util.ArrayList;
import java.util.List;

public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece> HELD_FUEL = new ArrayList<>(List.of());

    public static boolean hasFuel() {
        return !HELD_FUEL.isEmpty();
    }

    public static void update() {
        updateGamePieces();
        SimulatedGamePiece.logAll();
    }

    private static void updateGamePieces() {
        updateCollection();
        updateHeldFuelPoses();
        updateEjection();
    }

    private static void updateCollection() {
        final Translation3d robotRelativeCollectionPosition = SimulatedGamePieceConstants.COLLECTION_CHECK_POSITION;
        final Translation3d collectionPose = robotRelativeToFieldRelative(robotRelativeCollectionPosition);

        // Assumes Intake is active (you may need to add folding logic checks here if the intake is stowed)
        if (isCollectingFuel() && HELD_FUEL.size() < SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL) {
            final ArrayList<SimulatedGamePiece> collectedFuel = getCollectedFuel(collectionPose);
            for (SimulatedGamePiece fuel : collectedFuel) {
                if (HELD_FUEL.size() >= SimulatedGamePieceConstants.MAXIMUM_HELD_FUEL)
                    return;
                addHeldFuel(fuel);
            }
        }
    }

    public static void addHeldFuel(SimulatedGamePiece fuel) {
        HELD_FUEL.add(fuel);
        fuel.resetIndexing();
    }

    private static ArrayList<SimulatedGamePiece> getCollectedFuel(Translation3d collectionPosition) {
        final ArrayList<SimulatedGamePiece> collectedFuel = new ArrayList<>();
        for (SimulatedGamePiece gamePiece : SimulatedGamePiece.getUnheldGamePieces())
            if (gamePiece.getDistanceFromPositionMeters(collectionPosition) <= SimulatedGamePieceConstants.INTAKE_TOLERANCE_METERS)
                collectedFuel.add(gamePiece);
        return collectedFuel;
    }

    private static boolean isCollectingFuel() {
        return RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.POWERED_OPEN);
    }

    private static void updateEjection() {
        if (hasFuel()) {
            final SimulatedGamePiece ejectableFuel = getEjectableFuel();
            if (ejectableFuel != null) {
                ejectGamePiece(ejectableFuel);
            }
        }
    }

    private static SimulatedGamePiece getEjectableFuel() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed()) continue;

            // In a dumper setup, pieces in row 0 are the ones currently being ejected
            if (heldFuel.getIndexerGridSlot().getX() == 0) {
                return heldFuel;
            }
        }
        return null;
    }

    private static void ejectGamePiece(SimulatedGamePiece ejectedGamePiece) {
        int exitColumn = (int) ejectedGamePiece.getIndexerGridSlot().getY();

        ejectedGamePiece.release();
        HELD_FUEL.remove(ejectedGamePiece);

        for (SimulatedGamePiece piece : HELD_FUEL) {
            piece.release();
            piece.resetIndexing();
        }

        CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(ejectedGamePiece, exitColumn));
    }

    private static void updateHeldFuelPoses() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed()) {
                heldFuel.resetIndexing();
            }

            if (heldFuel.getIndexerGridSlot() != null)
                heldFuel.updatePosition(calculateHeldFuelFieldRelativePosition(heldFuel.getIndexerGridSlot()));
        }
    }

    private static Translation3d calculateHeldFuelFieldRelativePosition(Translation2d gridSlot) {
        // Calculate offset based on Row (X) and Col (Y)
        // Center the 4 columns around the robot's Y axis
        double colOffset = (gridSlot.getY() - (SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY - 1) / 2.0) * SimulatedGamePieceConstants.INDEXER_COL_SPACING_METERS;
        double rowOffset = gridSlot.getX() * -SimulatedGamePieceConstants.INDEXER_ROW_SPACING_METERS; // Negative goes back into the robot

        Translation3d indexerOffset = new Translation3d(rowOffset, colOffset, 0);

        // Use the new indexer subsystem pose calculation
        final Pose3d robotRelativeIndexerPose = IndexerConstants.FUEL_IN_INDEXER_POSE;

        final Transform3d fuelOffsetFromIndexerPose = new Transform3d(
                indexerOffset,
                new Rotation3d()
        );

        return robotRelativeToFieldRelative(robotRelativeIndexerPose.plus(fuelOffsetFromIndexerPose).getTranslation());
    }

    private static Translation3d robotRelativeToFieldRelative(Translation3d robotRelativePose) {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        return robotPose.plus(new Transform3d(robotRelativePose, new Rotation3d())).getTranslation();
    }
}