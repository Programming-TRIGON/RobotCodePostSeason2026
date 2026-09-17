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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            masterAngleMotor = IntakeConstants.MASTER_ANGLE_MOTOR,
            masterIntakeMotor = IntakeConstants.MASTER_INTAKE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            IntakeConstants.DEFAULT_MAXIMUM_VELOCITY,
            IntakeConstants.DEFAULT_MAXIMUM_ACCELERATION
    ).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

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
        IntakeConstants.INTAKE_MOTOR_MECHANISM.update(masterIntakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));

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
        IntakeConstants.FOLLOWER_ANGLE_MOTOR.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        masterAngleMotor.update();
        IntakeConstants.FOLLOWER_ANGLE_MOTOR.update();
        masterIntakeMotor.update();
        IntakeConstants.FOLLOWER_INTAKE_MOTOR.update();
        IntakeConstants.ANGLE_ENCODER.update();
        Logger.recordOutput("Intake/CurrentArmAngle", getCurrentAngle().getDegrees());
        Logger.recordOutput("Intake/TargetArmAngle", targetAngle.getDegrees());
    }

    @Override
    public void stop() {
        masterAngleMotor.stopMotor();
        masterIntakeMotor.stopMotor();
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
    }

    public boolean isLimitSwitchPressed() {
        return !IntakeConstants.INTAKE_LIMIT_SWITCH.get();
    }

    public boolean atState(IntakeConstants.IntakeState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    @AutoLogOutput(key = "Intake/IntakeAtTargetAngle")
    public boolean atTargetState() {
        return Math.abs(targetState.targetAngle.minus(getCurrentAngle()).getDegrees()) < IntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    boolean atAngle(Rotation2d angle){
        return Math.abs(angle.minus(getCurrentAngle()).getDegrees()) < IntakeConstants.ANGLE_TOLERANCE.getDegrees();
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
        masterIntakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        masterAngleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    @AutoLogOutput(key = "Intake/isIntakeStuckOnHopper")
    boolean isIntakeStuckOnHopper() {
        return masterAngleMotor.getSignal(TalonFXSignal.STATOR_CURRENT) > IntakeConstants.INTAKE_ASSIST_CURRENT_THRESHOLD;
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = IntakeConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = IntakeConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.POSITION));
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, IntakeConstants.MAXIMUM_ANGLE.minus(getCurrentAngle()).getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }
}