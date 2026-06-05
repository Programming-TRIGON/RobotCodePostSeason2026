package frc.trigon.robot.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.lib.hardware.misc.KeyboardController;
import frc.trigon.lib.hardware.misc.XboxController;
import frc.trigon.robot.RobotContainer;

import java.util.function.DoubleUnaryOperator;

public class OperatorConstants {
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.07;
    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int
            DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT = 1,
            DRIVER_CONTROLLER_LEFT_STICK_EXPONENT = 2;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_RIGHT_STICK_EXPONENT, DRIVER_CONTROLLER_LEFT_STICK_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();
    private static final double ARE_CAMERAS_DISCONNECTED_CHECK_DEBOUNCE_SECONDS = 3;

    public static final double
            POV_DIVIDER = 2,
            TRANSLATION_STICK_SPEED_DIVIDER = 1,
            ROTATION_STICK_SPEED_DIVIDER = 1;

    public static final double MINIMUM_VELOCITY_TOWARDS_GAME_PIECE_FOR_INTAKE_ASSIST_METERS_PER_SECOND = 1;
    private static final double
            INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT = 60,
            INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_SLOPE = -15;
    public static final DoubleUnaryOperator INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA =
            x -> MathUtil.clamp(
                    (INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_SLOPE * x) + INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT,
                    0,
                    INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA_INTERCEPT
            );

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            CAMERAS_DISCONNECTED_TRIGGER = new Trigger(() -> !RobotContainer.ROBOT_POSE_ESTIMATOR.hasUpdateFromCameras()).debounce(ARE_CAMERAS_DISCONNECTED_CHECK_DEBOUNCE_SECONDS);
    public static final Trigger
            SHOOTING_TRIGGER = DRIVER_CONTROLLER.rightBumper().or(OPERATOR_CONTROLLER.d()),
            CLOSE_TO_HUB_SETPOINT_SETTER_TRIGGER = DRIVER_CONTROLLER.povDown().or(OPERATOR_CONTROLLER.i()),
            RIGHT_TREMCG_SETTER_TRIGGER = DRIVER_CONTROLLER.povRight().or(OPERATOR_CONTROLLER.l()),
            LEFT_TRENCH_SETTER_TRIGGER = DRIVER_CONTROLLER.povLeft().or(OPERATOR_CONTROLLER.j()),
            TOP_RIGHT_SETTER_TRIGGER = DRIVER_CONTROLLER.povDownRight().or(OPERATOR_CONTROLLER.o()),
            TOP_LEFT_TOWER_SETTER_TRIGGER = DRIVER_CONTROLLER.povDownLeft().or(OPERATOR_CONTROLLER.u()),
            FIXED_SHOOTING_TRIGGER = DRIVER_CONTROLLER.leftStick().or(OPERATOR_CONTROLLER.s()),
            FIXED_DELIVERY_TRIGGER = DRIVER_CONTROLLER.rightStick().or(OPERATOR_CONTROLLER.f()),
            INTAKE_TRIGGER = DRIVER_CONTROLLER.leftTrigger().or(OPERATOR_CONTROLLER.v()),
            PRELOAD_TRIGGER = OPERATOR_CONTROLLER.f(),
            CLOSE_INTAKE_TRIGGER = DRIVER_CONTROLLER.leftBumper().or(OPERATOR_CONTROLLER.z()),
            CLOSE_INTAKE_WHILE_SHOOTING_TRIGGER = CLOSE_INTAKE_TRIGGER.and(SHOOTING_TRIGGER),
            CLOSE_INTAKE_WITHOUT_SHOOTING_TRIGGER = CLOSE_INTAKE_TRIGGER.and(SHOOTING_TRIGGER.negate());
}