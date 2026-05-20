package frc.trigon.robot.misc.shootingcalculations.shootingvisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

import java.util.Random;

public class VisualizeFuelShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final Random RANDOM = new Random();

    private final SimulatedGamePiece shotFuel;
    private Translation3d currentFuelVelocity;

    // ToF Calibration Trackers
    private double simulatedFlightTimeSeconds = 0;
    private boolean hasLoggedScore = false;

    public VisualizeFuelShootingCommand(SimulatedGamePiece shotFuel) {
        this.shotFuel = shotFuel;
    }

    public static InstantCommand getScheduleShotCommand(SimulatedGamePiece shotFuel) {
        return new InstantCommand(() -> CommandScheduler.getInstance().schedule(new VisualizeFuelShootingCommand(shotFuel)));
    }

    @Override
    public void initialize() {
        shotFuel.updatePosition(SHOOTING_CALCULATIONS.calculateCurrentFuelExitPose());
        currentFuelVelocity = calculateFuelExitVelocityVector();
        simulatedFlightTimeSeconds = 0;
        hasLoggedScore = false;
    }

    @Override
    public void execute() {
        if (isScoredInHub() && !hasLoggedScore) {
            // This prints the ToF to your console so you can use it in your interpolation map!
            System.out.println("[Sim Calibration] Shot scored! Simulated Time of Flight: " + simulatedFlightTimeSeconds + "s");
            hasLoggedScore = true;
            ejectFromHub();
        }

        int iterations = (int) (RobotHardwareStats.getPeriodicTimeSeconds() / FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS);
        for (int i = 0; i < iterations; i++) {
            stepSimulation();
            simulatedFlightTimeSeconds += FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS;
        }
    }

    @Override
    public boolean isFinished() {
        return shotFuel.getPosition().getZ() < FuelShootingVisualizationConstants.END_SIMULATION_HEIGHT_METERS && currentFuelVelocity.getZ() < 0;
    }

    @Override
    public void end(boolean interrupted) {
        final Translation3d currentPosition = shotFuel.getPosition();
        shotFuel.updatePosition(new Translation3d(currentPosition.getX(), currentPosition.getY(), SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters));
    }

    private Translation3d calculateFuelExitVelocityVector() {
        final Translation3d shootingVelocityVector = calculateShootingVelocityVector();
        final Translation3d robotVelocityVector = new Translation3d(RobotContainer.SWERVE.getFieldRelativeVelocity());
        return shootingVelocityVector.plus(robotVelocityVector);
    }

    private Translation3d calculateShootingVelocityVector() {
        // Removed lookup table scaling; assume Shooter RPM is calibrated to exit m/s.
        final double fuelExitSpeedMetersPerSecond = RobotContainer.SHOOTER.getCurrentVelocityMetersPerSecond();

        final Rotation2d dumperPitch = RobotContainer.HOOD.getCurrentPitch();
        final Rotation2d chassisFieldRelativeAngle = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();

        return new Translation3d(fuelExitSpeedMetersPerSecond, new Rotation3d(0, -dumperPitch.getRadians(), chassisFieldRelativeAngle.getRadians()));
    }

    private void stepSimulation() {
        final Translation3d gravitySpeedVector = new Translation3d(0, 0, -FuelShootingVisualizationConstants.G_FORCE * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS);
        final Translation3d dragSpeedVector = calculateCurrentDragSpeedVector(currentFuelVelocity);

        // Removed Magnus Lift since drum shooters do not apply asymmetric backspin
        currentFuelVelocity = currentFuelVelocity.plus(gravitySpeedVector).plus(dragSpeedVector);

        final Translation3d currentGamePiecePosition = shotFuel.getPosition();
        shotFuel.updatePosition(currentGamePiecePosition.plus(currentFuelVelocity.times(FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS)));
    }

    private Translation3d calculateCurrentDragSpeedVector(Translation3d currentGamePieceVelocity) {
        final double velocityMagnitude = currentGamePieceVelocity.getNorm();
        if (velocityMagnitude < 1e-6) return new Translation3d();
        final double dragForceMagnitude = 0.5 * FuelShootingVisualizationConstants.AIR_DENSITY * velocityMagnitude * velocityMagnitude * FuelShootingVisualizationConstants.DRAG_COEFFICIENT * FuelShootingVisualizationConstants.GAME_PIECE_AREA;
        final double dragAccelerationMagnitude = dragForceMagnitude / FuelShootingVisualizationConstants.GAME_PIECE_MASS_KG;
        final double dragVelocityMagnitude = dragAccelerationMagnitude * FuelShootingVisualizationConstants.SIMULATION_TIME_STEP_SECONDS;
        return currentGamePieceVelocity.div(velocityMagnitude).times(-dragVelocityMagnitude);
    }

    private void ejectFromHub() {
        final Translation3d ejectionPower = new Translation3d(getRandomNumber(SimulatedGamePieceConstants.EJECTION_FROM_HUB_MINIMUM_VELOCITY_METERS_PER_SECOND, SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_VELOCITY_METERS_PER_SECOND), 0, 0);
        final Rotation3d ejectionRotation = new Rotation3d(0, 0, Units.degreesToRadians(getRandomNumber(-SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_ANGLE.getDegrees(), SimulatedGamePieceConstants.EJECTION_FROM_HUB_MAXIMUM_ANGLE.getDegrees())) + (Flippable.isRedAlliance() ? Math.PI : 0));
        shotFuel.updatePosition(SimulatedGamePieceConstants.EJECT_FUEL_FROM_HUB_POSITION.get());
        currentFuelVelocity = ejectionPower.rotateBy(ejectionRotation);
    }

    private boolean isScoredInHub() {
        return shotFuel.getPosition().getDistance(SimulatedGamePieceConstants.SCORE_CHECK_POSITION.get()) < SimulatedGamePieceConstants.SCORE_TOLERANCE_METERS;
    }

    private double getRandomNumber(double min, double max) {
        return min + (max - min) * RANDOM.nextDouble();
    }
}