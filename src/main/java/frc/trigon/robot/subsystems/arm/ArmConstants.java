package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.lib.utilities.mechanisms.SingleJointedArmMechanism2d;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class ArmConstants {
    private static final int MOTOR_ID = 15;
    private static final int FOLLOWER_ID = 16;
    private static final String MOTOR_NAME = "ArmMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final TalonFXMotor FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_ID, "FollowerArmMotor");

    static final boolean FOC_ENABLED = true;
    static final double GEAR_RATIO = ShooterConstants.BOTTOM_WHEEL_GEAR_RATIO * 5;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            ARM_MASS_KILOGRAMS = 2,
            ARM_LENGTH_METERS = 0.3;
    static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90), // TODO: placeholder, set real max angle
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0); // TODO: placeholder, set real min angle
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    private static final String MECHANISM_NAME = "ArmMechanism";
    private static final Color MECHANISM_COLOR = Color.kOrange;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            MECHANISM_NAME,
            ARM_LENGTH_METERS,
            MECHANISM_COLOR
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.7).per(Units.Seconds),
            Units.Volts.of(1),
            null
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1);

    public enum ArmState {
        OPEN(MAXIMUM_ANGLE),
        CLOSE(MINIMUM_ANGLE);

        public final Rotation2d targetAngle;

        ArmState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }

    static {
        configureMotor();
        configureFollowerAngleMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 35 : 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.2 : 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 2.5 : 0;
        config.Slot0.kA = 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.15 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 20 : 3;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 20 : 2;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

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

    private static void configureFollowerAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MOTOR.getID(), MotorAlignmentValue.Opposed);
        FOLLOWER_MOTOR.setControl(followerRequest);

        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }
}