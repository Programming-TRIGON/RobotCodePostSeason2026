package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Indexer extends MotorSubsystem {
    private final TalonFXMotor motor = IndexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(5).withEnableFOC(IndexerConstants.FOC_ENABLED);

    public Indexer() {
        setName("Indexer");
    }

    @Override
    public void updateMechanism() {
        IndexerConstants.MECHANISM.update(
                motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)
        );
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    void setTargetState(IndexerConstants.IndexerState targetState) {
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

}
