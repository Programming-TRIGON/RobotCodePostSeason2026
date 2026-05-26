package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            masterAngleMotor = IntakeConstants.MASTER_ANGLE_MOTOR,
            intakeMotor = IntakeConstants.INTAKE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            IntakeConstants.DEFAULT_MAXIMUM_VELOCITY,
            IntakeConstants.DEFAULT_MAXIMUM_ACCELERATION
            )
            .withEnableFOC
            (
            IntakeConstants.FOC_ENABLED
    );
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IntakeAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(masterAngleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterAngleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_ANGLE_MOTOR_MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        IntakeConstants.INTAKE_MOTOR_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));

        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        masterAngleMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        masterAngleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        masterAngleMotor.update();
        IntakeConstants.FOLLOWER_ANGLE_MOTOR.update();
        intakeMotor.update();
        IntakeConstants.ANGLE_ENCODER.update();
        Logger.recordOutput("Intake/CurrentArmAngle", getCurrentAngle());
    }

    @Override
    public void stop() {
        masterAngleMotor.stopMotor();
        intakeMotor.stopMotor();
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
    }

    public boolean atState(IntakeConstants.IntakeState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    public boolean atTargetState() {
        return targetState.targetAngle.minus(getCurrentAngle()).getDegrees() < IntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetVoltage, targetState.targetAngle, targetState.speedScalar);
    }

    void setTargetState(double targetVoltage, Rotation2d targetAngle, double speedScalar) {
        setTargetVoltage(targetVoltage);
        setTargetAngle(targetAngle);
        scalePositionRequestSpeed(speedScalar);
    }

    void setTargetVoltage(double targetVoltage) {
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(targetVoltage);
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        masterAngleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = IntakeConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = IntakeConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.POSITION)).minus(IntakeConstants.VISUALIZATION_OFFSET);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -getCurrentAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }
}