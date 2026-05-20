package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_ANGLE_MECHANISM.update(
                getCurrentIntakeArmAngle()
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
        Logger.recordOutput("Intake/CurrentArmVoltage", getCurrentIntakeArmAngle());
    }

    @Override
    public void stop() {
        masterAngleMotor.stopMotor();
        intakeMotor.stopMotor();
        IntakeConstants.INTAKE_MOTOR_MECHANISM.setTargetVelocity(0);
        IntakeConstants.INTAKE_ANGLE_MECHANISM.setTargetAngle(Rotation2d.fromDegrees(90));
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetIntakeVoltage, targetState.targetIntakeArmAngle);
    }

    void setTargetState(double targetIntakeVoltage, Rotation2d targetIntakeArmAngle) {
        setTargetIntakeVoltage(targetIntakeVoltage);
        setTargetIntakeArmAngle(targetIntakeArmAngle);
    }

    private void setTargetIntakeVoltage(double targetIntakeVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetIntakeVoltage));
    }

    private void setTargetIntakeArmAngle(Rotation2d targetIntakeArmAngle) {
        masterAngleMotor.setControl(positionRequest.withPosition(targetIntakeArmAngle.getRotations()));
    }

    private Rotation2d getCurrentIntakeArmAngle() {
        return Rotation2d.fromRotations(masterAngleMotor.getSignal(TalonFXSignal.POSITION));
    }

    private double getCurrentIntakeVoltage() {
        return intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -getCurrentIntakeArmAngle().getRadians(), 0)
        );
        return IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(pitchTransform);
    }
}

