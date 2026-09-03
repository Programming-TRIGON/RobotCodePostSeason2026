package frc.trigon.robot.subsystems.loader;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class LoaderConstants {
    private static final int MOTOR_ID = 15;
    private static final String MOTOR_NAME = "LoaderMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 1.35;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(3),
            null
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 10;
    private static final String LOADER_MECHANISM_NAME = "LoaderMechanism";
    static final SpeedMechanism2d LOADER_MECHANISM = new SpeedMechanism2d(
            LOADER_MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    public static final double EJECT_FROM_INTAKE_VELOCITY = -2;
    public static final double EJECT_FROM_SHOOTER_VELOCITY = 2;
    public static final double LOAD_FOR_SHOOTING_VOLTAGE_THRESHOLD = 1;
    public static final double EJECT_FROM_INTAKE_VOLTAGE_THRESHOLD = -1;
    static final double WHEEL_DIAMETER_METERS = 0.05;
    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.2;

    static {
        configureLoaderMotor();
    }

    private static void configureLoaderMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 15 : Loader.metersToRotations(10);
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 60.0 : Loader.metersToRotations(15);
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }
}