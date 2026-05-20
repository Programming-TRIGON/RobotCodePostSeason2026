package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            masterAngleMotor = IntakeConstants.MASTER_ANGLE_MOTOR,
            intakeMotor = IntakeConstants.INTAKE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLE);
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_ANGLE_MECHANISM.update(
                getCurrentIntakeArmVoltage()
        );
        IntakeConstants.INTAKE_MOTOR_MECHANISM.update(
                getCurrentIntakeVoltage()
        );

        Logger.recordOutput("Poses/Components/IntakePose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        masterAngleMotor.update();
        intakeMotor.update();
        Logger.recordOutput("Intake/CurrentVoltage", getCurrentIntakeArmVoltage());
    }

    @Override
    public void stop() {
        masterAngleMotor.stopMotor();
        intakeMotor.stopMotor();
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
        IntakeConstants.INTAKE_ANGLE_MECHANISM.setTargetVelocity(0);
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetIntakeVoltage, targetState.targetIntakeArmVoltage);
    }

    void setTargetState(double targetIntakeVoltage, double targetIntakeArmVoltage) {
        setTargetIntakeVoltage(targetIntakeVoltage);
        setTargetIntakeArmVoltage(targetIntakeArmVoltage);
    }

    private void setTargetIntakeVoltage(double targetIntakeVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetIntakeVoltage));
    }

    private void setTargetIntakeArmVoltage(double targetIntakeArmVoltage) {
        masterAngleMotor.setControl(voltageRequest.withOutput(targetIntakeArmVoltage));
    }

    private double getCurrentIntakeArmVoltage() {
        return masterAngleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private double getCurrentIntakeVoltage() {
        return intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -getCurrentArmAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }

    private Rotation2d getCurrentArmAngle() {
        return Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.POSITION));
    }
}

