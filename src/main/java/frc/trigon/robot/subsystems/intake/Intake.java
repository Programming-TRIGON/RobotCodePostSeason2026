package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor masterAngleMotor = IntakeConstants.MASTER_ANGLE_MOTOR;
    private final TalonFXMotor followerAngleMotor = IntakeConstants.FOLLOWER_ANGLE_MOTOR;
    private final TalonFXMotor intakeMotor = IntakeConstants.INTAKE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLE);
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_ANGLE_MECHANISM.update(
                getCurrentArmAngle(),
                Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        IntakeConstants.INTAKE_MOTOR_MECHANISM.update(
                getCurrentWheelVoltage()
        );

        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        masterAngleMotor.update();
        followerAngleMotor.update();
        Logger.recordOutput("Intake/CurrentVoltage", getCurrentIntakeVoltage());
    }

    @Override
    public void stop() {
    masterAngleMotor.stopMotor();
    intakeMotor.stopMotor();
    IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetWheelVoltage, targetState.targetIntakeVoltage);
    }

    void setTargetState(double targetWheelVoltage, double targetIntakeVoltage) {
        setTargetWheelVoltage(targetWheelVoltage);
        setTargetIntakeVoltage(targetIntakeVoltage);
    }

    private void setTargetWheelVoltage(double targetWheelVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetWheelVoltage));
    }

    private void setTargetIntakeVoltage(double targetIntakeVoltage) {
        masterAngleMotor.setControl(voltageRequest.withOutput(targetIntakeVoltage));
    }

    private double getCurrentIntakeVoltage() {
        return masterAngleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private double getCurrentWheelVoltage() {
        return intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0,-getCurrentArmAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }

    private Rotation2d getCurrentArmAngle() {
        return  Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.POSITION));
    }
}

