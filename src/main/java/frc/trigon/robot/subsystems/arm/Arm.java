package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends MotorSubsystem {
    private final TalonFXMotor motor = ArmConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private ArmConstants.ArmState targetState = ArmConstants.ArmState.CLOSE;

    public Arm() {
        setName("Arm");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("ArmMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        final Rotation2d currentAngle = getCurrentAngle();
        final Rotation2d targetProfiledAngle = getTargetProfiledAngle();
        ArmConstants.MECHANISM.update(
                currentAngle,
                targetProfiledAngle
        );
    }

    @Override
    public void updatePeriodically() {
        motor.update();

        final Rotation2d currentAngle = getCurrentAngle();
        final Rotation2d targetProfiledAngle = getTargetProfiledAngle();
        Logger.recordOutput("Arm/TargetState", targetState.toString());
        Logger.recordOutput("Arm/TargetAngleDegrees", targetState.targetAngle.getDegrees());
        Logger.recordOutput("Arm/CurrentAngleDegrees", currentAngle.getDegrees());
        Logger.recordOutput("Arm/TargetProfiledAngleDegrees", targetProfiledAngle.getDegrees());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage).withIgnoreSoftwareLimits(false));
    }

    @AutoLogOutput(key = "Arm/AtTargetState")
    public boolean atTargetState() {
        return atAngle(targetState.targetAngle);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - getCurrentAngle().getDegrees()) < ArmConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public ArmConstants.ArmState getTargetState() {
        return targetState;
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION));
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        this.targetState = targetState;
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    void resetTargetVoltage() {
        motor.setControl(voltageRequest.withOutput(-1).withIgnoreSoftwareLimits(true));
    }

    void resetPosition() {
        motor.setPosition(ArmConstants.MINIMUM_ANGLE.getRotations());
        motor.stopMotor();
    }

    private Rotation2d getTargetProfiledAngle() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }
}