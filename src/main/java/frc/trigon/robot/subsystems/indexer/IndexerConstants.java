package frc.trigon.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSMotor;
import frc.trigon.lib.hardware.phoenix6.talonfxs.TalonFXSSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;

public class IndexerConstants {
    private static final int MOTOR_ID = 13;
    private static final String MOTOR_NAME = "IndexerMotor";
    static final TalonFXSMotor MOTOR = new TalonFXSMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 4;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEAR_BOX = DCMotor.getMinion(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEAR_BOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double MAXIMUM_DISPLAYABLE_VOLTAGE = 12;
    private static final String MECHANISM_NAME = "IndexerMechanism";
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            MECHANISM_NAME,
            MAXIMUM_DISPLAYABLE_VOLTAGE
    );

    public static final Pose3d FUEL_IN_INDEXER_POSE = new Pose3d(
            new Translation3d(-0.04721098, 0, 0.156750004),
            new Rotation3d()
    );
    public static final double LOAD_FOR_SHOOTING_VOLTAGE_THRESHOLD = 1;
    public static final double EJECT_FROM_INTAKE_VOLTAGE_THRESHOLD = -1;

    static {
        final TalonFXSConfiguration config = new TalonFXSConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.ExternalFeedback.withSensorToMechanismRatio(GEAR_RATIO);
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSSignal.STATOR_CURRENT, 100);
    }

    public enum IndexerState {
        LOAD_FOR_SHOOTING(2),
        LOAD_FOR_DELIVERY(2),
        PRELOAD(1),
        AGITATE(1),
        EJECT_FROM_INTAKE(-2),
        EJECT_FROM_SHOOTER(2),
        REST(0);

        public final double targetVoltage;

        IndexerState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}
