package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.misc.shootingcalculations.shootingvisualization.VisualizeFuelShootingCommand;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece> HELD_FUEL = new ArrayList<>(List.of());
    private static final double INDEXER_RAMP_ANGLE_RADS = Math.toRadians(19.8);

    public static boolean hasFuel() {
        return !HELD_FUEL.isEmpty();
    }

    public static void update() {
        updateGamePieces();
        SimulatedGamePiece.logAll();
    }

    /**
     * Teleports the simulated robot so that the Shooter Exit is an exact distance away from a target.
     * Use this purely for rapid simulation calibration of the ShootingMap.
     */
    public static void teleportRobotForCalibration(ShootingCalculations.TargetLocation targetLocation, double exactShooterDistanceMeters) {
        final Translation2d targetPos = targetLocation.position.get();
        final Rotation2d robotRotation = new Rotation2d();
        final Translation2d shooterOffsetFromChassis = ShooterConstants.FUEL_EXIT_SHOOTER_POSE.getTranslation().toTranslation2d();

        final Translation2d newRobotPosition = targetPos
                .minus(new Translation2d(exactShooterDistanceMeters, 0))
                .minus(shooterOffsetFromChassis);

        RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(new Pose2d(newRobotPosition, robotRotation));
    }

    private static void updateGamePieces() {
        updateCollection();
        updateHeldFuelPoses();
        updateEjection();
    }

    private static void updateCollection() {
        final Translation3d robotRelativeCollectionPosition = SimulatedGamePieceConstants.COLLECTION_CHECK_POSITION;
        final Translation3d collectionPose = robotRelativeToFieldRelative(robotRelativeCollectionPosition);

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
        if (hasFuel() && isShootingFuel()) {
            final List<SimulatedGamePiece> ejectableFuels = getEjectableFuels();
            if (!ejectableFuels.isEmpty()) {
                ejectGamePieces(ejectableFuels);
            }
        }
    }

    private static boolean isShootingFuel() {
        return (RobotContainer.LOADER.atState(LoaderConstants.LoaderState.LOAD_FOR_DELIVERY) || RobotContainer.LOADER.atState(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)) &&
                (RobotContainer.INDEXER.atState(IndexerConstants.IndexerState.LOAD_FOR_DELIVERY) || RobotContainer.INDEXER.atState(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING));
    }

    private static List<SimulatedGamePiece> getEjectableFuels() {
        final List<SimulatedGamePiece> ejectable = new ArrayList<>();
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                continue;

            // Only fire the balls that managed to reach the absolute front of the line
            if (heldFuel.getIndexerGridSlot().getX() == 0)
                ejectable.add(heldFuel);
        }
        return ejectable;
    }

    private static void ejectGamePieces(List<SimulatedGamePiece> ejectedGamePieces) {
        for (SimulatedGamePiece piece : ejectedGamePieces) {
            int exitColumn = (int) piece.getIndexerGridSlot().getY();
            piece.release();
            HELD_FUEL.remove(piece);

            CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(piece, exitColumn));
        }

        for (SimulatedGamePiece piece : HELD_FUEL) {
            piece.release();
            piece.resetIndexing();
        }
    }

    private static void updateHeldFuelPoses() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                heldFuel.resetIndexing();

            if (heldFuel.getIndexerGridSlot() != null)
                heldFuel.updatePosition(calculateHeldFuelFieldRelativePosition(heldFuel.getIndexerGridSlot()));
        }
    }

    private static Translation3d calculateHeldFuelFieldRelativePosition(Translation2d gridSlot) {
        int row = (int) gridSlot.getX();
        int col = (int) gridSlot.getY();

        // 1. Calculate the exact mathematical offsets
        double yOffset = calculateWidthOffsetY(col);
        Translation2d profileOffsetXZ = calculateProfileOffsetXZ(row);

        Translation3d exactOffset = new Translation3d(profileOffsetXZ.getX(), yOffset, profileOffsetXZ.getY());

        // 2. Apply visual adjustments
        Translation3d scatteredOffset = applyOrganicScatter(exactOffset, row, col);

        // 3. Transform to Field Space
        return convertIndexerOffsetToFieldRelative(scatteredOffset);
    }

    /**
     * Calculates the Left/Right (Y) offset to center the 4 columns around the robot's center.
     */
    private static double calculateWidthOffsetY(int col) {
        double centerOffset = (SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY - 1) / 2.0;
        return (col - centerOffset) * SimulatedGamePieceConstants.INDEXER_COL_SPACING_METERS;
    }

    /**
     * Calculates the Depth (X) and Height (Z) offset based on the ramp and stacking logic.
     * Returned as a Translation2d where X = Depth and Y = Height.
     */
    private static Translation2d calculateProfileOffsetXZ(int row) {
        boolean isStacked = row >= 4;
        int effectiveRow = isStacked ? row - 2 : row;

        Translation2d baseOffset = getBaseProfileOffset(effectiveRow);

        if (isStacked) {
            return baseOffset.plus(getStackingShift());
        }

        return baseOffset;
    }

    /**
     * Calculates the position of a game piece assuming it is resting directly on the physical
     * plastic of the loader (flat) or the indexer (ramp).
     */
    private static Translation2d getBaseProfileOffset(int effectiveRow) {
        double spacing = SimulatedGamePieceConstants.INDEXER_ROW_SPACING_METERS;

        if (effectiveRow <= 1) {
            // Flat Section (Loader)
            return new Translation2d(effectiveRow * -spacing, 0);
        }

        // Ramp Section (Indexer)
        double flatDist = 1 * -spacing;
        double rampDist = (effectiveRow - 1) * spacing;

        double xOffset = flatDist - (rampDist * Math.cos(INDEXER_RAMP_ANGLE_RADS));
        double zOffset = rampDist * Math.sin(INDEXER_RAMP_ANGLE_RADS);

        return new Translation2d(xOffset, zOffset);
    }

    /**
     * Calculates the perpendicular offset required to stack a ball on top of another ball
     * that is resting on the 19.8 degree ramp.
     */
    private static Translation2d getStackingShift() {
        double stackHeight = SimulatedGamePieceConstants.FUEL_DIAMETER_METERS * 0.85; // nestle slightly

        double xShift = -stackHeight * Math.sin(INDEXER_RAMP_ANGLE_RADS);
        double zShift = stackHeight * Math.cos(INDEXER_RAMP_ANGLE_RADS);

        return new Translation2d(xShift, zShift);
    }

    /**
     * Applies a deterministic random scatter to simulate a messy pile of game pieces.
     */
    private static Translation3d applyOrganicScatter(Translation3d baseOffset, int row, int col) {
        Random scatterRNG = new Random(row * 100L + col);
        double xScatter = (scatterRNG.nextDouble() - 0.5) * 0.04;
        double yScatter = (scatterRNG.nextDouble() - 0.5) * 0.04;

        return new Translation3d(
                baseOffset.getX() + xScatter,
                baseOffset.getY() + yScatter,
                baseOffset.getZ()
        );
    }

    /**
     * Takes the local 3D offset inside the indexer and translates it into global field coordinates.
     */
    private static Translation3d convertIndexerOffsetToFieldRelative(Translation3d indexerOffset) {
        final Pose3d robotRelativeIndexerPose = IndexerConstants.FUEL_IN_INDEXER_POSE;

        final Transform3d fuelOffsetFromIndexerPose = new Transform3d(
                indexerOffset,
                new Rotation3d()
        );

        Translation3d robotRelativeFuelPosition = robotRelativeIndexerPose.plus(fuelOffsetFromIndexerPose).getTranslation();
        return robotRelativeToFieldRelative(robotRelativeFuelPosition);
    }

    private static Translation3d robotRelativeToFieldRelative(Translation3d robotRelativePose) {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        return robotPose.plus(new Transform3d(robotRelativePose, new Rotation3d())).getTranslation();
    }
}