package frc.trigon.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends MotorSubsystem {
    private final TalonFXMotor motor = KickerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(KickerConstants.FOC_ENABLED);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(KickerConstants.FOC_ENABLED);
    private KickerConstants.KickerState targetState;
    private double targetVelocityMetersPerSecond = 0;

    public Kicker() {
        setName("Loader");
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("KickerMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return KickerConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        motor.stopMotor();
        KickerConstants.KICKER_MECHANISM.setTargetVelocity(0);
        targetVelocityMetersPerSecond = 0;
    }

    @Override
    public void updateMechanism() {
        KickerConstants.KICKER_MECHANISM.update(
                getCurrentVelocityMetersPerSecond(),
                getTargetProfiledVelocityMetersPerSecond()
        );
    }

    @Override
    public void updatePeriodically() {
        motor.update();

        Logger.recordOutput("Kicker/TargetVelocityMetersPerSecond", targetVelocityMetersPerSecond);
    }

    @AutoLogOutput(key = "Kicker/Voltage")
    public double getCurrentVoltage() {
        return motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    @AutoLogOutput(key = "Loader/AtTargetVelocity")
    public boolean atTargetVelocity() {
        return atVelocity(targetVelocityMetersPerSecond);
    }

    public boolean atState(KickerConstants.KickerState targetState) {
        return atVelocity(targetState.targetVelocityMetersPerSecond) && this.targetState == targetState;
    }

    public boolean atVelocity(double targetVelocityMetersPerSecond) {
        return Math.abs(getCurrentVelocityMetersPerSecond() - targetVelocityMetersPerSecond) < KickerConstants.VELOCITY_TOLERANCE_METERS_PER_SECOND;
    }

    void setTargetState(KickerConstants.KickerState targetState) {
        this.targetState = targetState;
        setTargetVelocity(targetState.targetVelocityMetersPerSecond);
    }

    void setTargetVelocity(double targetVelocityMetersPerSecond) {
        motor.setControl(velocityRequest.withVelocity(metersToRotations(targetVelocityMetersPerSecond)));
        this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
    }

    @AutoLogOutput(key = "Kicker/TargetVelocityMetersPerSecond")
    private double getCurrentVelocityMetersPerSecond() {
        return rotationsToMeters(motor.getSignal(TalonFXSignal.VELOCITY));
    }

    @AutoLogOutput(key = "Kicker/TargetProfiledVelocityMetersPerSecond")
    private double getTargetProfiledVelocityMetersPerSecond() {
        return rotationsToMeters(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    static double rotationsToMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, KickerConstants.ANGULAR_TO_LINEAR_CONVERSION_FACTOR);
    }

    static double metersToRotations(double meters) {
        return Conversions.distanceToRotations(meters, KickerConstants.ANGULAR_TO_LINEAR_CONVERSION_FACTOR);
    }
}
