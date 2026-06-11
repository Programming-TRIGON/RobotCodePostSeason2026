package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.utilities.flippable.Flippable;
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
    private static double lastEjectionTimestampSeconds = 0;

    public static boolean hasFuel() {
        return !HELD_FUEL.isEmpty();
    }

    /**
     * Authoritative held-state check: a piece is held if and only if it is in the HELD_FUEL list,
     * regardless of whether it currently has a hopper cell allocated.
     */
    public static boolean isHeld(SimulatedGamePiece fuel) {
        return HELD_FUEL.contains(fuel);
    }

    public static void update() {
        updateGamePieces();
        SimulatedGamePiece.logAll();
    }

    /**
     * Teleports the simulated robot so that the Shooter Exit is an exact distance away from a target.
     * Use this purely for rapid simulation calibration of the ShootingMap.
     */
    public static void teleportRobotForSimulationShootingMapCalibration(ShootingCalculations.TargetShootingLocation targetShootingLocation, double exactShooterDistanceMeters) {
        final Translation2d targetPos = targetShootingLocation.position.get();
        final boolean shouldFlip = Flippable.isRedAlliance() != targetShootingLocation.isDelivery;
        final Rotation2d newRobotRotation = shouldFlip ? Rotation2d.kZero : Rotation2d.k180deg;

        final Translation2d shooterOffsetFromChassis = ShooterConstants.FUEL_EXIT_SHOOTER_POSE.getTranslation().toTranslation2d().rotateBy(newRobotRotation);

        final Translation2d newRobotPosition = targetPos
                .minus(new Translation2d(shouldFlip ? -exactShooterDistanceMeters : exactShooterDistanceMeters, 0))
                .minus(shooterOffsetFromChassis);

        RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(new Pose2d(newRobotPosition, newRobotRotation));
    }

    private static void updateGamePieces() {
        updateCollection();
        updateHeldFuelPoses();
        updateEjection();
        updateIntakeEjection();
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
        if (!hasFuel() || !isShootingFuel())
            return;

        final double now = Timer.getFPGATimestamp();
        final double fireInterval = isIntakeClosed()
                ? SimulatedGamePieceConstants.SHOOTING_INTERVAL_INTAKE_CLOSED_SECONDS
                : SimulatedGamePieceConstants.SHOOTING_INTERVAL_INTAKE_OPEN_SECONDS;

        if (now - lastEjectionTimestampSeconds < fireInterval)
            return;

        final List<SimulatedGamePiece> ejectableFuels = getEjectableFuels();
        if (ejectableFuels.isEmpty())
            return;

        // Fire the whole front row at once (up to one ball per exit lane), then re-settle the pile.
        ejectGamePieces(ejectableFuels);
        lastEjectionTimestampSeconds = now;
    }

    /**
     * Handles ejecting fuel backward through the intake: when the loader, indexer, and intake are
     * all running in reverse, all held fuel is released and dropped at the intake's collection
     * position so pieces appear at the front of the robot rather than at the shooter exit.
     */
    private static void updateIntakeEjection() {
        if (!hasFuel() || !isEjectingThroughIntake())
            return;

        final Translation3d ejectPosition = robotRelativeToFieldRelative(SimulatedGamePieceConstants.COLLECTION_CHECK_POSITION);
        final ArrayList<SimulatedGamePiece> toEject = new ArrayList<>(HELD_FUEL);
        for (SimulatedGamePiece piece : toEject) {
            piece.release();
            piece.updatePosition(ejectPosition);
            HELD_FUEL.remove(piece);
        }
    }

    /**
     * Returns true when the robot is actively reverse-ejecting through the intake:
     * loader, indexer, and intake all spinning backwards.
     */
    private static boolean isEjectingThroughIntake() {
        return RobotContainer.LOADER.getCurrentVoltage() < LoaderConstants.LoaderState.EJECT_FROM_INTAKE.targetVoltage
                && RobotContainer.INDEXER.getCurrentVoltage() < IndexerConstants.IndexerState.EJECT_FROM_INTAKE.targetVoltage
                && RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.POWERED_OPEN);
    }

    /**
     * When the intake is closed (POWERED_CLOSE) the fuel feeds straight to the shooter without
     * fighting the intake, so it fires much faster, mirroring the real robot.
     */
    private static boolean isIntakeClosed() {
        return RobotContainer.INTAKE.atState(IntakeConstants.IntakeState.POWERED_CLOSE);
    }

    private static boolean isShootingFuel() {
        return RobotContainer.LOADER.getCurrentVoltage() > LoaderConstants.LOAD_FOR_SHOOTING_VOLTAGE_THRESHOLD
                && RobotContainer.INDEXER.getCurrentVoltage() > IndexerConstants.LOAD_FOR_SHOOTING_VOLTAGE_THRESHOLD;
    }

    private static List<SimulatedGamePiece> getEjectableFuels() {
        final List<SimulatedGamePiece> ejectable = new ArrayList<>();
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                continue;

            final SimulatedGamePiece.HopperCell cell = heldFuel.getHopperCell();
            // Only fire the balls sitting at the very front (shooter-feed) row of the bottom layer.
            if (cell.layerIndex() == 0 && cell.depthIndex() == 0)
                ejectable.add(heldFuel);
        }
        return ejectable;
    }

    private static void ejectGamePieces(List<SimulatedGamePiece> ejectedGamePieces) {
        for (SimulatedGamePiece piece : ejectedGamePieces) {
            final int exitColumn = mapWidthIndexToExitColumn(piece.getHopperCell().widthIndex());
            piece.release();
            HELD_FUEL.remove(piece);

            CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(piece, exitColumn));
        }

        // Re-settle the remaining pile so balls collapse forward/down into the freed cells.
        for (SimulatedGamePiece remaining : HELD_FUEL) {
            remaining.release();
            remaining.resetIndexing();
        }
    }

    /**
     * Maps a hopper width cell to one of the shooter's exit lanes by RELATIVE position across the
     * two widths, so a wider hopper spreads evenly across the lanes instead of dumping every extra
     * right-side column into the last lane. Reduces to identity when the widths match.
     */
    private static int mapWidthIndexToExitColumn(int widthIndex) {
        final int hopperWidth = SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY;
        final int exitLanes = SimulatedGamePieceConstants.INDEXER_WIDTH_CAPACITY;

        if (hopperWidth <= 1 || exitLanes <= 1)
            return 0;

        // Normalize the hopper column to [0, 1], scale to the exit-lane span, and round to a lane.
        final double relativePosition = (double) widthIndex / (hopperWidth - 1);
        final int lane = (int) Math.round(relativePosition * (exitLanes - 1));

        return Math.max(0, Math.min(lane, exitLanes - 1));
    }

    private static void updateHeldFuelPoses() {
        for (SimulatedGamePiece heldFuel : HELD_FUEL) {
            if (!heldFuel.isIndexed())
                heldFuel.resetIndexing();

            if (heldFuel.getHopperCell() != null)
                heldFuel.updatePosition(calculateHeldFuelFieldRelativePosition(heldFuel.getHopperCell()));
        }
    }

    /**
     * Converts a discrete hopper cell into a field-relative position by:
     * 1. computing the cell's offset inside the hopper volume (flat front rows, sloped rear rows),
     * 2. applying a small deterministic scatter so the pile looks organic,
     * 3. transforming the robot-relative offset into field coordinates.
     */
    private static Translation3d calculateHeldFuelFieldRelativePosition(SimulatedGamePiece.HopperCell cell) {
        final Translation3d exactOffset = calculateHopperCellOffset(cell);
        final Translation3d scatteredOffset = applyOrganicScatter(exactOffset, cell);
        return convertHopperOffsetToFieldRelative(scatteredOffset);
    }

    /**
     * Builds the robot-relative offset for a cell:
     * <p>
     * - Width (Y) is centered so the pile is symmetric about the robot center.
     * <p>
     * - Rows climb the ~19 degree indexer ramp as the depth index grows, EXCEPT the rearmost
     * FLAT_BACK_ROW_COUNT rows, which hold the height of the last sloped row to form a flat shelf.
     * <p>
     * - Stacked layers nest into the pockets between the balls below: each higher layer is shifted
     * half a cell sideways (alternating per layer), nudged slightly in depth, and raised by less
     * than a full layer height, so the pile looks settled instead of balls floating straight up.
     * <p>
     * Finally the pile is flipped by HOPPER_DEPTH_DIRECTION and shifted by HOPPER_ANCHOR_OFFSET.
     */
    private static Translation3d calculateHopperCellOffset(SimulatedGamePiece.HopperCell cell) {
        final double widthCenter = (SimulatedGamePieceConstants.HOPPER_WIDTH_CAPACITY - 1) / 2.0;
        double yOffset = (cell.widthIndex() - widthCenter) * SimulatedGamePieceConstants.HOPPER_WIDTH_SPACING_METERS;

        final double depthSpacing = SimulatedGamePieceConstants.HOPPER_DEPTH_SPACING_METERS;
        final double rampCos = SimulatedGamePieceConstants.INDEXER_RAMP_ANGLE.getCos();
        final double rampSin = SimulatedGamePieceConstants.INDEXER_RAMP_ANGLE.getSin();

        // Depth index at which the ramp stops climbing and the flat back shelf begins.
        final int lastClimbingRow = SimulatedGamePieceConstants.HOPPER_DEPTH_CAPACITY
                - SimulatedGamePieceConstants.FLAT_BACK_ROW_COUNT - 1;

        // Number of ramp steps this row has climbed (capped at the flat-shelf start).
        final int climbSteps = Math.min(cell.depthIndex(), Math.max(lastClimbingRow, 0));
        final double alongRamp = climbSteps * depthSpacing;

        // Depth that continues advancing along the floor even on the flat shelf.
        double xOffset = alongRamp * rampCos;
        double zOffset = alongRamp * rampSin;
        if (cell.depthIndex() > lastClimbingRow) {
            final int flatSteps = cell.depthIndex() - lastClimbingRow;
            xOffset += flatSteps * depthSpacing; // shelf extends back at constant height
        }

        // Natural nesting for stacked layers.
        if (cell.layerIndex() > 0) {
            final double rise = cell.layerIndex() * SimulatedGamePieceConstants.HOPPER_LAYER_SPACING_METERS
                    * SimulatedGamePieceConstants.NEST_VERTICAL_FACTOR;
            zOffset += rise;

            // Alternate the sideways nestle direction each layer so the stack doesn't lean.
            final double sideSign = (cell.layerIndex() % 2 == 1) ? 1.0 : -1.0;
            yOffset += sideSign * SimulatedGamePieceConstants.NEST_WIDTH_SHIFT_METERS;
            xOffset += sideSign * SimulatedGamePieceConstants.NEST_DEPTH_SHIFT_METERS;
        }

        // Flip onto the hopper side, nudge the pile snug against the shooter wheels, and apply
        // the tunable anchor offset. The nudge acts opposite the fill direction so the front row
        // seats against the wheels. With the extra depth row, the pile now physically reaches them.
        final double snugNudge = SimulatedGamePieceConstants.SHOOTER_WHEEL_SNUG_NUDGE_METERS;
        xOffset = (xOffset - snugNudge) * SimulatedGamePieceConstants.HOPPER_DEPTH_DIRECTION + SimulatedGamePieceConstants.HOPPER_ANCHOR_OFFSET.getX();
        final double finalY = yOffset + SimulatedGamePieceConstants.HOPPER_ANCHOR_OFFSET.getY();
        final double finalZ = zOffset + SimulatedGamePieceConstants.HOPPER_ANCHOR_OFFSET.getZ();

        return new Translation3d(xOffset, finalY, finalZ);
    }

    /**
     * Applies a deterministic random scatter so the pile reads as loose balls rather than a
     * perfect lattice. Vertical scatter is omitted so balls never appear to float.
     */
    private static Translation3d applyOrganicScatter(Translation3d baseOffset, SimulatedGamePiece.HopperCell cell) {
        final Random scatterRNG = new Random(cell.layerIndex() * 10000L + cell.depthIndex() * 100L + cell.widthIndex());
        final double xScatter = (scatterRNG.nextDouble() - 0.5) * SimulatedGamePieceConstants.ORGANIC_SCATTER_METERS;
        final double yScatter = (scatterRNG.nextDouble() - 0.5) * SimulatedGamePieceConstants.ORGANIC_SCATTER_METERS;

        return new Translation3d(
                baseOffset.getX() + xScatter,
                baseOffset.getY() + yScatter,
                baseOffset.getZ()
        );
    }

    /**
     * Adds the local hopper offset to the indexer anchor and converts to field coordinates.
     * <p>
     * Only the anchor pose's TRANSLATION is used. The offset is kept in clean robot axes
     * (X forward, Y left, Z up), so the pile's orientation is fully defined by this code's own
     * ramp math and never skews if FUEL_IN_INDEXER_POSE happens to carry a rotation.
     */
    private static Translation3d convertHopperOffsetToFieldRelative(Translation3d hopperOffset) {
        final Translation3d anchorTranslation = IndexerConstants.FUEL_IN_INDEXER_POSE.getTranslation();
        final Translation3d robotRelativeFuelPosition = anchorTranslation.plus(hopperOffset);
        return robotRelativeToFieldRelative(robotRelativeFuelPosition);
    }

    private static Translation3d robotRelativeToFieldRelative(Translation3d robotRelativePose) {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        return robotPose.plus(new Transform3d(robotRelativePose, new Rotation3d())).getTranslation();
    }
}