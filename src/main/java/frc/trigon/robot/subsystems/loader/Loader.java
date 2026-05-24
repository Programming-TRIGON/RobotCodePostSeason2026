package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Loader extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = LoaderConstants.MASTER_MOTOR,
            followerMotor = LoaderConstants.FOLLOWER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(LoaderConstants.FOC_ENABLED);

    public Loader() {
        setName("Loader");
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }

    @Override
    public void updateMechanism() {
        LoaderConstants.LOADER_MECHANISM.update(
                masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE),
                followerMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)
        );
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        followerMotor.update();
    }

    void setTargetState(LoaderConstants.LoaderState targetState) {
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
        followerMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}
