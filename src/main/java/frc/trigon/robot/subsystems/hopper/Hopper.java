package frc.trigon.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends MotorSubsystem {
    public final TalonFXSMotor motor = HopperConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(HopperConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, HopperConstants.DEFAULT_MAXIMUM_VELOCITY, HopperConstants.DEFAULT_MAXIMUM_ACCELERATION);
    private double targetPositionRotations = 0;
    private HopperConstants.HopperState targetState;

    public Hopper() {
        setName("Hopper");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("HopperMotor")
                .linearPosition(Units.Meters.of(getPositionMeters()))
                .linearVelocity(Units.MetersPerSecond.of(motor.getSignal(TalonFXSSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        HopperConstants.MECHANISM.updateCurrentPosition(
                getPositionMeters()
        );
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return HopperConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        Logger.recordOutput("Hopper/CurrentPositionMeters", getPositionMeters());
        Logger.recordOutput("Hopper/TargetPositionMeters", rotationsToMeters(targetPositionRotations));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean atState(HopperConstants.HopperState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    void setTargetState(HopperConstants.HopperState targetState) {
        this.targetState = targetState;
        setTargetPositionMeters(targetState.targetPositionMeters);
    }

    void setTargetPositionMeters(double targetPositionMeters) {
        setTargetPositionRotations(metersToRotations(targetPositionMeters));
    }

    @AutoLogOutput(key = "Hopper/HopperAtTargetMetersPosition")
    boolean atTargetState() {
        return Math.abs(targetState.targetPositionMeters - getPositionMeters()) < HopperConstants.POSITION_METERS_TOLERANCE;
    }

    double getPositionMeters() {
        return rotationsToMeters(getPositionRotations());
    }

    double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, HopperConstants.DRUM_DIAMETER_METERS);
    }

    private double getPositionRotations() {
        return motor.getSignal(TalonFXSSignal.POSITION);
    }

    private void setTargetPositionRotations(double targetPositionRotations) {
        this.targetPositionRotations = targetPositionRotations;
        motor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private double metersToRotations(double positionMeters) {
        return Conversions.distanceToRotations(positionMeters, HopperConstants.DRUM_DIAMETER_METERS);
    }
}
