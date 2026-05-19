package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class IndexerConstants {
    private static final int MOTOR_ID = 1;
    private static final String MOTOR_NAME = "IndexerMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 4;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.001;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEAR_BOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(4),
            null
    );

    static final Pose3d VISUALIZATION_ORIGIN_POSE = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final String MECHANISM_NAME = "IndexerMechanism";
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.2;
    static final double SIMULATION_SLIPPAGE_COMPENSATION_MULTIPLIER = 1 / 4.0;
    static final double LOADING_SPEED_RELATIVE_TO_SHOOTING_COEFFICIENT = 1;

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 10 : 6.73362886 / (5 / 9.0);
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 60 : 243.689458 / (5 / 9.0);

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum IndexerState {
        LOAD_FOR_DELIVERY(10),
        LOAD_FOR_EJECT(5),
        UNJAM(-10),
        STOP(0);

        public final double targetVelocityMetersPerSecond;

        IndexerState(double targetVelocityMetersPerSecond) {
            this.targetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
        }
    }
}
