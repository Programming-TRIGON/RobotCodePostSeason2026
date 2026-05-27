package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Indexer extends MotorSubsystem {
    private final TalonFXSMotor motor = IndexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IndexerConstants.FOC_ENABLED);

    public Indexer() {
        setName("Indexer");
    }

    @Override
    public void updateMechanism() {
        IndexerConstants.MECHANISM.update(motor.getSignal(TalonFXSSignal.MOTOR_VOLTAGE));
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        IndexerConstants.MECHANISM.setTargetVelocity(0);
    }

    void setTargetState(IndexerConstants.IndexerState targetState) {
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        IndexerConstants.MECHANISM.setTargetVelocity(targetVoltage);
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}
