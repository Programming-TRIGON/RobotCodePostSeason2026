package frc.trigon.robot.misc.shootingcalculations;

import frc.trigon.lib.hardware.RobotHardwareStats;

public class ShootingCalculationsConstants {
    static final double POSE_PREDICTION_TIME_SECONDS = RobotHardwareStats.isSimulation() ? 0.02 : 0.08;
    static final int VIRTUAL_HUB_CALCULATION_ITERATIONS = 5;
}