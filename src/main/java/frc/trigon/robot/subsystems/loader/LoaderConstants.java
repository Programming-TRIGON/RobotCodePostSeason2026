package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class LoaderConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14;
    private static final String
            MASTER_MOTOR_NAME = "LoaderMasterMotor",
            FOLLOWER_MOTOR_NAME = "LoaderFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 4;
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Aligned;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double MAXIMUM_DISPLAYABLE_VOLTAGE = 12;
    private static final String LOADER_MECHANISM_NAME = "LoaderMechanism";
    static final SpeedMechanism2d LOADER_MECHANISM = new SpeedMechanism2d(
            LOADER_MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VOLTAGE
    );

    static {
        configureLoaderMasterMotor();
        configureLoaderFollowerMotor();
    }

    private static void configureLoaderMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureLoaderFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        FOLLOWER_MOTOR.applyConfiguration(config);
        FOLLOWER_MOTOR.setPhysicsSimulation(SIMULATION);

        final Follower followRequest = new Follower(MASTER_MOTOR.getID(), FOLLOWER_ALIGNMENT_TO_MASTER);
        FOLLOWER_MOTOR.setControl(followRequest);

        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    public enum LoaderState {
        LOAD_FOR_DELIVERY(6),
        LOAD_FOR_SHOOTING(4),
        LOAD_FOR_EJECTION(4),
        PRELOAD(2),
        REST(0);

        public final double targetVoltage;

        LoaderState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}
