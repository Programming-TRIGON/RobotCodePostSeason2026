package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

import static frc.trigon.robot.subsystems.swerve.SwerveCommands.getClosedLoopFieldRelativeDriveCommand;

public class ShootingCommands {

    private static final FlippableRotation2d HUB_ANGLE =
            FlippableRotation2d.fromDegrees(90, true);

    private static final FlippableRotation2d RIGHT_DELIVERY_ANGLE =
            FlippableRotation2d.fromDegrees(70, true);

    private static final FlippableRotation2d LEFT_DELIVERY_ANGLE =
            FlippableRotation2d.fromDegrees(110, true);

    public static Command getShootOrDeliverCommand() {
        return new ConditionalCommand(
                getShootToHubCommand(),
                getShootForDeliveryCommand(),
                FieldConstants::isRobotInAllianceZone
        );
    }

    private static Command getShootToHubCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getAimAtHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                getDriveToHubAngleCommand(),
                GeneralCommands.runWhen(
                        getLoadForShootingCommand(),
                        ShootingCommands::isReadyToShoot
                )
        );
    }

    private static Command getShootForDeliveryCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getAimForDeliveryCommand(),
                HoodCommands.getAimForDeliveryCommand(),
                getDriveToDeliveryAngleCommand(),
                GeneralCommands.runWhen(
                        getLoadForShootingCommand(),
                        ShootingCommands::isReadyToShoot
                )
        );
    }

    public static Command getLoadForShootingCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
        );
    }

    private static Command getDriveToHubAngleCommand() {
        return getClosedLoopFieldRelativeDriveCommand(
                OperatorConstants.DRIVER_CONTROLLER::getLeftX,
                OperatorConstants.DRIVER_CONTROLLER::getLeftY,
                () -> HUB_ANGLE
        );
    }

    private static Command getDriveToDeliveryAngleCommand() {
        return getClosedLoopFieldRelativeDriveCommand(
                OperatorConstants.DRIVER_CONTROLLER::getLeftX,
                OperatorConstants.DRIVER_CONTROLLER::getLeftY,
                ShootingCommands::getDeliveryAngle
        );
    }

    private static FlippableRotation2d getDeliveryAngle() {
        return FieldConstants.isRobotInAllianceZone()
                ? RIGHT_DELIVERY_ANGLE
                : LEFT_DELIVERY_ANGLE;
    }

    private static boolean isReadyToShoot() {
        return RobotContainer.SHOOTER.atTargetVelocity()
                && RobotContainer.HOOD.atTargetAngle()
                && RobotContainer.SWERVE.atAngle(HUB_ANGLE);
    }
}