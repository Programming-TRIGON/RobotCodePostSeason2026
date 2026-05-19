package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Indexer extends MotorSubsystem {
    //private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    //change after nahum hahamud gomer
    private final TalonFXMotor motor = IndexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IndexerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(IndexerConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond;

    public Indexer(){
        setName("Indexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IndexerMotor")
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IndexerConstants.MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IndexerConstants.SYSID_CONFIG;
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetVelocityMetersPerSecond = 0;
    }

    public boolean atTargetState(IndexerConstants.IndexerState targetState) {
        return atVelocity(targetState.targetVelocityMetersPerSecond);
    }

    public boolean atVelocity(double velocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - velocityMetersPerSecond)
                <= IndexerConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    public Pose3d calculateComponentPose() {
        final Transform3d yawTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)).getRadians() * IndexerConstants.SIMULATION_SLIPPAGE_COMPENSATION_MULTIPLIER)
        );
        return IndexerConstants.VISUALIZATION_ORIGIN_POSE.transformBy(yawTransform);
    }

    void loadToShooter() {
        //final double targetShooterVelocityFromShootingCalculations = shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond();
        //final double targetLoadingVelocity = targetShooterVelocityFromShootingCalculations * IndexerConstants.LOADING_SPEED_RELATIVE_TO_SHOOTING_COEFFICIENT;
        //change after nahum hahamud gomer
        setTargetVelocity(0);
    }

    void setTargetState(IndexerConstants.IndexerState targetState) {
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    private double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }
}
