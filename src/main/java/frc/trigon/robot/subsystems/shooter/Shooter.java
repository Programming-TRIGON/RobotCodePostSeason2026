package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private final TalonFXMotor motor = ShooterConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ShooterConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(ShooterConstants.FOC_ENABLED);
    private double targetVelocityMetersPerSecond = 0;
    private boolean isAimingAtHub = true;

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("ShooterMasterMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(getCurrentVelocityMetersPerSecond()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetVelocityMetersPerSecond = 0;
        isAimingAtHub = false;
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        ShooterConstants.FOLLOWER_MOTOR.update();
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ShooterConstants.SYSID_CONFIG;
    }

    @Override
    public void updateMechanism() {
        final double currentVelocityMetersPerSecond = getCurrentVelocityMetersPerSecond();
        final double targetProfiledVelocityMetersPerSecond = motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE);
        ShooterConstants.MECHANISM.update(
                currentVelocityMetersPerSecond,
                targetProfiledVelocityMetersPerSecond
        );

        Logger.recordOutput("Shooter/CurrentVelocityMetersPerSecond", currentVelocityMetersPerSecond);
        Logger.recordOutput("Shooter/TargetVelocityMetersPerSecond", this.targetVelocityMetersPerSecond);
        Logger.recordOutput("Shooter/TargetProfiledVelocityMetersPerSecond", targetProfiledVelocityMetersPerSecond);
    }

    @AutoLogOutput(key = "Shooting/Conditions/ShooterAtTargetVelocity")
    public boolean atTargetVelocity() {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) < ShooterConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    public double getCurrentVelocityMetersPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    public double getTargetVelocityMetersPerSecond() {
        return targetVelocityMetersPerSecond;
    }

    void getSetTargetVelocity(double targetVelocityMetersPerSecond) {
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        motor.setControl(velocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    void aimAtHub() {}
}