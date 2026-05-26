package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class ShooterConstants {
    private static final int
            MASTER_MOTOR_ID = 15,
            FOLLOWER_MOTOR_ID = 16;
    private static final String
            MASTER_MOTOR_NAME = "ShooterMasterMotor",
            FOLLOWER_MOTOR_NAME = "ShooterFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    static final double
            TOP_WHEEL_GEAR_RATIO = 2.0,
            BOTTOM_WHEEL_GEAR_RATIO = 1.0;
    static final double AVERAGE_GEAR_RATIO =
            (TOP_WHEEL_GEAR_RATIO + BOTTOM_WHEEL_GEAR_RATIO) / 2.0;
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT_TO_MASTER = MotorAlignmentValue.Opposed;
    private static final double STATOR_CURRENT_LIMIT_AMPS = 60;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.002;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(GEARBOX, AVERAGE_GEAR_RATIO, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(2).per(Units.Second),
            Units.Volts.of(4),
            null
    );

    private static final String MECHANISM_NAME = "ShooterMechanism";
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 15;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double
            TOP_WHEEL_DIAMETER = 0.1016,
            BOTTOM_WHEEL_DIAMETER = 0.05;
    static final double EFFECTIVE_WHEEL_DIAMETER =
            AVERAGE_GEAR_RATIO * 0.5 * ((TOP_WHEEL_DIAMETER / TOP_WHEEL_GEAR_RATIO) + (BOTTOM_WHEEL_DIAMETER / BOTTOM_WHEEL_GEAR_RATIO));
    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.2;

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 1 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.019646 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.12055 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.0092472 : 0;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 15.0 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 60.0 : 0;

        config.Feedback.SensorToMechanismRatio = AVERAGE_GEAR_RATIO;
        config.Feedback.VelocityFilterTimeConstant = 0.03;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.SUPPLY_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_ALIGNMENT_TO_MASTER);
        FOLLOWER_MOTOR.setControl(followRequest);
    }
}