package frc.trigon.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class HoodConstants {
    private static final int MOTOR_ID = 17;
    private static final String MOTOR_NAME = "HoodMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 30;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(MOTOR_AMOUNT);
    private static final double
            HOOD_MASS_KILOGRAMS = 2,
            HOOD_LENGTH_METERS = 0.258;
    private static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(52),
            MINIMUM_ANGLE = Rotation2d.fromDegrees(20);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            HOOD_LENGTH_METERS,
            HOOD_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    private static final String MECHANISM_NAME = "HoodMechanism";
    private static final Color MECHANISM_COLOR = Color.kYellow;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            HOOD_LENGTH_METERS,
            MECHANISM_COLOR
    );
    static final Pose3d HOOD_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.2758, 0, 0.45400412),
            new Rotation3d(0, MINIMUM_ANGLE.getRadians(), 0)
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.1).per(Units.Seconds),
            Units.Volts.of(0.3),
            null
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1);
    static final Rotation2d
            REST_ANGLE = Rotation2d.fromDegrees(20),
            DELIVERY_ANGLE = Rotation2d.fromDegrees(50),
            EJECTION_ANGLE = Rotation2d.fromDegrees(52);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 35 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.22942 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016146 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 2.6669 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.041586 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.18316 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 20 : 0;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 20 : 0;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_ANGLE.getRotations();
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MINIMUM_ANGLE.getRotations();

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }
}