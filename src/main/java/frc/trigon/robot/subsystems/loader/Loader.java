package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Loader extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor masterMotor = LoaderConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(LoaderConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(LoaderConstants.FOC_ENABLED);
    private LoaderConstants.LoaderState targetState;
    private double targetVelocityMetersPerSecond = 0;

    public Loader() {
        setName("Loader");
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        masterMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("LoaderMotor")
                .angularPosition(Units.Rotations.of(masterMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return LoaderConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        LoaderConstants.LOADER_MECHANISM.setTargetVelocity(0);
        targetVelocityMetersPerSecond = 0;
    }

    @Override
    public void updateMechanism() {
        LoaderConstants.LOADER_MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                getTargetProfiledVelocityMetersPerSecond()
        );
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();

        Logger.recordOutput("Loader/TargetVelocityMetersPerSecond", targetVelocityMetersPerSecond);
    }

    @AutoLogOutput(key = "Loader/CurrentVelocityMetersPerSecond")
    public double getCurrentVelocityMetersPerSecond() {
        return rotationsToMeters(masterMotor.getSignal(TalonFXSignal.VELOCITY));
    }

    @AutoLogOutput(key = "Loader/AtTargetVelocity")
    public boolean atTargetVelocity() {
        return atVelocity(targetVelocityMetersPerSecond);
    }

    public boolean atVelocity(double targetVelocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) < LoaderConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    void aimForShooting() {
        final double targetVelocityMetersPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityMetersPerSecond() * LoaderConstants.TARGET_PERCENTAGE_OF_SHOOTER_VELOCITY;
        setTargetVelocity(targetVelocityMetersPerSecond);
    }

    void aimForFixedShooting(double shooterTargetVelocityMetersPerSecond) {
        final double targetVelocityMetersPerSecond = shooterTargetVelocityMetersPerSecond * LoaderConstants.TARGET_PERCENTAGE_OF_SHOOTER_VELOCITY;
        setTargetVelocity(targetVelocityMetersPerSecond);
    }

    void setTargetState(LoaderConstants.LoaderState targetState) {
        this.targetState = targetState;
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        targetVelocityMetersPerSecond = MathUtil.clamp(
                targetVelocityMetersPerSecond,
                -LoaderConstants.MAXIMUM_LOADER_VELOCITY_METERS_PER_SECOND,
                LoaderConstants.MAXIMUM_LOADER_VELOCITY_METERS_PER_SECOND
        );
        masterMotor.setControl(velocityRequest.withVelocity(metersToRotations(targetVelocityMetersPerSecond)));
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
    }

    @AutoLogOutput(key = "Loader/TargetProfiledVelocityMetersPerSecond")
    private double getTargetProfiledVelocityMetersPerSecond() {
        return rotationsToMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    static double rotationsToMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, LoaderConstants.WHEEL_DIAMETER_METERS);
    }

    static double metersToRotations(double meters) {
        return Conversions.distanceToRotations(meters, LoaderConstants.WHEEL_DIAMETER_METERS);
    }
}