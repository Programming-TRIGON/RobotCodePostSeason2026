package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SpeedMechanism2d;
import frc.trigon.robot.constants.RobotConstants;

public class IntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            MASTER_ANGLE_MOTOR_ID = 10,
            FOLLOWER_ANGLE_MOTOR_ID = 11;
    private static final String
            INTAKE_MOTOR_NAME = "IntakeMotor",
            MASTER_ANGLE_MOTOR_NAME = "IntakeMasterAngleMotor",
            FOLLOWER_ANGLE_MOTOR_NAME = "IntakeFollowerAngleMotor";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            MASTER_ANGLE_MOTOR = new TalonFXMotor(MASTER_ANGLE_MOTOR_ID, MASTER_ANGLE_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_ANGLE_MOTOR = new TalonFXMotor(FOLLOWER_ANGLE_MOTOR_ID, FOLLOWER_ANGLE_MOTOR_NAME, RobotConstants.CANIVORE_NAME);

    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 60,
            INTAKE_MOTOR_GEAR_RATIO = 1.55;
    static final boolean FOC_ENABLE = true;
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
    private static final boolean SHOULD_ARM_SIMULATE_GRAITY = true;
    private static final double WHEEL_MOMENT_OF_INERTIA = 0.003;
    static final SingleJointedArmSimulation INTAKE_ANGLE_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_ARM_SIMULATE_GRAITY
    );
    static final SimpleMotorSimulation INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            WHEEL_MOMENT_OF_INERTIA
    );

    private static String
            ANGLE_MOTOR_MECHANISM_NAME = "IntakeAngleMotorMechanism",
            INTAKE_MOTOR_MECHANISM_NAME = "IntakeWheelMotorMechanism";
    private static final double
            INTAKE_ANGLE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE = 12,
            INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE = 12;
    static final SpeedMechanism2d INTAKE_ANGLE_MECHANISM = new SpeedMechanism2d(
            ANGLE_MOTOR_MECHANISM_NAME,
            INTAKE_ANGLE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE
    );
    static final SpeedMechanism2d INTAKE_MOTOR_MECHANISM = new SpeedMechanism2d(
            INTAKE_MOTOR_MECHANISM_NAME,
            INTAKE_MOTOR_MAXIMUM_DISPLAYABLE_VOLTAGE
    );
    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    static {
        configureMasterAngleMotor();
        configureFollowerAngleMotor();
        configureIntakeMotor();
    }

    private static void configureMasterAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = 0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_MOTORS_CURRENT_LIMIT;

        MASTER_ANGLE_MOTOR.applyConfiguration(config);
        MASTER_ANGLE_MOTOR.setPhysicsSimulation(INTAKE_ANGLE_SIMULATION);

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

    public enum IntakeState {
        REST(0, 0),
        INTAKE(6, 0),
        LOADING(0, 6);

        public final double targetIntakeVoltage;
        public final double targetIntakeArmVoltage;

        IntakeState(double targetIntakeVoltage, double targetIntakeArmVoltage) {
            this.targetIntakeVoltage = targetIntakeVoltage;
            this.targetIntakeArmVoltage = targetIntakeArmVoltage;
        }
    }
}