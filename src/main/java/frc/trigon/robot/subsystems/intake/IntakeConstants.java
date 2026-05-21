package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;
import frc.trigon.robot.constants.RobotConstants;

public class IntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            MASTER_ANGLE_MOTOR_ID = 10,
            FOLLOWER_ANGLE_MOTOR_ID = 11,
            ANGLE_ENCODER_ID = 10;
    private static final String
            INTAKE_MOTOR_NAME = "IntakeMotor",
            MASTER_ANGLE_MOTOR_NAME = "IntakeMasterAngleMotor",
            FOLLOWER_ANGLE_MOTOR_NAME = "IntakeFollowerAngleMotor",
            ANGLE_ENCODER_NAME = "IntakeAngleEncoder";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            MASTER_ANGLE_MOTOR = new TalonFXMotor(MASTER_ANGLE_MOTOR_ID, MASTER_ANGLE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_ANGLE_MOTOR = new TalonFXMotor(FOLLOWER_ANGLE_MOTOR_ID, FOLLOWER_ANGLE_MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 60,
            INTAKE_MOTOR_GEAR_RATIO = 1.55;
    static final boolean FOC_ENABLED = true;
    private static final MotorAlignmentValue ANGLE_FOLLOWER_TO_MASTER = MotorAlignmentValue.Opposed;
    private static final double INTAKE_MOTOR_CURRENT_LIMIT = 40;
    private static final double ANGLE_MOTORS_CURRENT_LIMIT = 30;

    private static final int
            ANGLE_MOTOR_AMOUNT = 2,
            INTAKE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            ANGLE_GEARBOX = DCMotor.getFalcon500Foc(ANGLE_MOTOR_AMOUNT),
            INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(INTAKE_MOTOR_AMOUNT);
    private static final double
            INTAKE_LENGTH_METERS = 0.369,
            INTAKE_MASS_KILOGRAMS = 6;
    static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-15),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SHOULD_ARM_SIMULATE_GRAVITY = true;
    private static final double WHEEL_MOMENT_OF_INERTIA = 0.003;
    static final SingleJointedArmSimulation INTAKE_ANGLE_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAVITY
    );
    static final SimpleMotorSimulation INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            WHEEL_MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.8).per(Units.Seconds),
            Units.Volts.of(0.8),
            Units.Second.of(1000)
    );

    private static String
            ANGLE_MOTOR_MECHANISM_NAME = "IntakeAngleMotorMechanism",
            INTAKE_MOTOR_MECHANISM_NAME = "IntakeWheelMotorMechanism";
    private static final Color ANGLE_MOTOR_MECHANISM_COLOR = Color.kGreen;
    private static final double INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE = 12;
    static final SingleJointedArmMechanism2d INTAKE_ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            ANGLE_MOTOR_MECHANISM_NAME,
            INTAKE_LENGTH_METERS,
            ANGLE_MOTOR_MECHANISM_COLOR
    );
    static final SpeedMechanism2d INTAKE_MOTOR_MECHANISM = new SpeedMechanism2d(
            INTAKE_MOTOR_MECHANISM_NAME,
            INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE
    );
    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    static final Rotation2d ANGLE_MOTOR_TOLERANCE = Rotation2d.fromDegrees(2);

    static {
        configureMasterAngleMotor();
        configureFollowerAngleMotor();
        configureIntakeMotor();
        configureAngleEncoder();
    }

    private static void configureMasterAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 50 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.017109 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 6.9978 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.094634 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.18458 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 1 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 2 : 0;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTORS_CURRENT_LIMIT;

        MASTER_ANGLE_MOTOR.applyConfiguration(config);
        MASTER_ANGLE_MOTOR.setPhysicsSimulation(INTAKE_ANGLE_SIMULATION);

        MASTER_ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_ANGLE_MOTOR.registerThreadedSignal(TalonFXSignal.POSITION, 100);
    }

    private static void configureFollowerAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTORS_CURRENT_LIMIT;

        FOLLOWER_ANGLE_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_ANGLE_MOTOR.getID(), ANGLE_FOLLOWER_TO_MASTER);
        FOLLOWER_ANGLE_MOTOR.setControl(followerRequest);

        FOLLOWER_ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        FOLLOWER_ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        FOLLOWER_ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        FOLLOWER_ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        FOLLOWER_ANGLE_MOTOR.registerThreadedSignal(TalonFXSignal.POSITION, 100);
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = INTAKE_MOTOR_GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = INTAKE_MOTOR_CURRENT_LIMIT;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = 0;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(MASTER_ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    public enum IntakeState {
        REST(0, Rotation2d.fromDegrees(90)),
        INTAKE(6, Rotation2d.fromDegrees(-15)),
        LOADING(0, Rotation2d.fromDegrees(90));

        public final double targetIntakeVoltage;
        public final Rotation2d targetIntakeArmAngle;

        IntakeState(double targetIntakeVoltage, Rotation2d targetIntakeArmAngle) {
            this.targetIntakeVoltage = targetIntakeVoltage;
            this.targetIntakeArmAngle = targetIntakeArmAngle;
        }
    }
}