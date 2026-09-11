package frc.trigon.robot.subsystems.hopper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends MotorSubsystem {
    public final TalonFXMotor motor = HopperConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(HopperConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(HopperConstants.FOC_ENABLED);
    private HopperConstants.HopperState targetState = HopperConstants.HopperState.OPEN;

    public Hopper() {
        setName("Hopper");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("HopperMotor")
                .linearPosition(Units.Meters.of(getCurrentPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        HopperConstants.MECHANISM.updateMechanism(
                getCurrentPositionMeters(),
                getTargetProfiledPositionMeters(),
                Rotation2d.kZero,
                Rotation2d.kZero
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
        Logger.recordOutput("Hopper/TargetState", targetState);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean atState(HopperConstants.HopperState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    @AutoLogOutput(key = "Hopper/AtTargetState")
    public boolean atTargetState() {
        return Math.abs(targetState.targetPositionMeters - getCurrentPositionMeters()) < HopperConstants.TOLERANCE_METERS;
    }

    void setTargetState(HopperConstants.HopperState targetState) {
        this.targetState = targetState;
        setTargetPositionMeters(targetState.targetPositionMeters);
    }

    void setTargetPositionMeters(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(metersToRotations(targetPositionMeters)));
    }

    void resetTargetVoltage() {
        motor.setControl(voltageRequest.withOutput(HopperConstants.HOPPER_RESET_VOLTAGE).withIgnoreSoftwareLimits(true));
    }

    void resetPosition() {
        motor.setPosition(HopperConstants.RESET_POSITION_METERS);
        motor.stopMotor();
    }

    @AutoLogOutput(key = "Hopper/TargetProfiledPositionMeters")
    private double getTargetProfiledPositionMeters() {
        return rotationsToMeters(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    @AutoLogOutput(key = "Hopper/CurrentPositionMeters")
    private double getCurrentPositionMeters() {
        return rotationsToMeters(getCurrentPositionRotations());
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, HopperConstants.DRUM_DIAMETER_METERS);
    }

    private double metersToRotations(double positionMeters) {
        return Conversions.distanceToRotations(positionMeters, HopperConstants.DRUM_DIAMETER_METERS);
    }

    private double getCurrentPositionRotations() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }
}
