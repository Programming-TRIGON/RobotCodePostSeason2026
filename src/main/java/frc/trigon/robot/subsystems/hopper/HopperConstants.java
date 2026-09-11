package frc.trigon.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.Conversions;
import frc.trigon.lib.utilities.mechanisms.ArmElevatorMechanism2d;

public class HopperConstants {
    private static final int MOTOR_ID = 9;
    private static final String MOTOR_NAME = "HopperMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 11.25;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEAR_BOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(1),
            null
    );

    private static final String MECHANISM_NAME = "HopperMechanism";
    private static final double MAXIMUM_LENGTH_METERS = 0.3;
    private static final double MINIMUM_LENGTH_METERS = 0;
    private static final double STARTING_LENGTH_METERS = 0.1;

    private static final Color MECHANISM_COLOR = Color.kYellow;
    static final ArmElevatorMechanism2d MECHANISM = new ArmElevatorMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_LENGTH_METERS + STARTING_LENGTH_METERS,
            MINIMUM_LENGTH_METERS + STARTING_LENGTH_METERS,
            MECHANISM_COLOR
    );

    static final double DRUM_DIAMETER_METERS = 0.09144;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 8 : 2,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 8 : 2;
    static final double TOLERANCE_METERS = 0.01;
    static final double HOPPER_RESET_VOLTAGE = -1;
    static final double RESET_POSITION_METERS = MINIMUM_LENGTH_METERS;

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 50 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.6 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.0052251 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 1.0877 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.026632 : 0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.distanceToRotations(MAXIMUM_LENGTH_METERS, DRUM_DIAMETER_METERS);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.distanceToRotations(MINIMUM_LENGTH_METERS, DRUM_DIAMETER_METERS);

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);

    }

    public enum HopperState {
        OPEN(MAXIMUM_LENGTH_METERS),
        CLOSE(MINIMUM_LENGTH_METERS);

        public final double targetPositionMeters;

        HopperState(double targetPositionMeters) {
            this.targetPositionMeters = targetPositionMeters;
        }
    }
}
