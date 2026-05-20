package frc.trigon.robot.misc.shootingcalculations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.lib.hardware.RobotHardwareStats;

public class ShootingCalculationsConstants {
    static final Transform3d SHOOTER_TO_FUEL_EXIT = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    ); //TODO: get

    static final double POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0.08;
    static final int VIRTUAL_HUB_CALCULATION_ITERATIONS = 5;
}