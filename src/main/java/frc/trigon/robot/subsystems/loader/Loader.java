package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Loader extends MotorSubsystem {
    private final TalonFXMotor masterMotor = LoaderConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(LoaderConstants.FOC_ENABLED);
    private double targetVoltage = 0;

    public Loader() {
        setName("Loader");
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        targetVoltage = 0;
        LoaderConstants.LOADER_MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void updateMechanism() {
        LoaderConstants.LOADER_MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        Logger.recordOutput("Loader/CurrentVoltage", getCurrentVoltage());
        Logger.recordOutput("Loader/TargetVoltage", this.targetVoltage);
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        LoaderConstants.FOLLOWER_MOTOR.update();
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage).withIgnoreSoftwareLimits(false));
    }

    public double getCurrentVoltage() {
        return masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE);
    }

    void setTargetState(LoaderConstants.LoaderState targetState) {
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        this.targetVoltage = targetVoltage;
        LoaderConstants.LOADER_MECHANISM.setTargetVelocity(targetVoltage);
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}
