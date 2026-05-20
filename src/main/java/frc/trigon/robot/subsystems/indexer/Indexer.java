package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Indexer extends MotorSubsystem {
    //private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    //change after nahum hahamud gomer
    private final TalonFXMotor motor = IndexerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IndexerConstants.FOC_ENABLED);

    public Indexer() {
        setName("Indexer");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("SpindexerMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)));
        log.motor("IndexerMotor")
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        IndexerConstants.MECHANISM.update(
                getCurrentVoltage(),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IndexerConstants.SYSID_CONFIG;
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean atTargetState(IndexerConstants.IndexerState targetState) {
        return atVoltage(targetState.targetVoltage);
    }

    public boolean atVoltage(double voltage) {
        return Math.abs(getCurrentVoltage() - voltage)
                <= IndexerConstants.VOLTAGE_TOLERANCE;
    }

    void setTargetState(IndexerConstants.IndexerState targetState) {
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private double getCurrentVoltage() {
        return motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }
}
