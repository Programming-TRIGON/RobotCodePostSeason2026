package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.shootingcalculations.ShootingCalculations;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.indexer.IndexerCommands;
import frc.trigon.robot.subsystems.indexer.IndexerConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    public static Command getShootCommand() {
        return new ConditionalCommand(
                getShootAtHubCommand(),
                getShootForDeliveryCommand(),
                FieldConstants::isRobotInAllianceZone
        );
    }

    private static Command getShootAtHubCommand() {
        return new ParallelCommandGroup(
                getAimAtHubCommand(),
                GeneralCommands.runWhen(
                        getLoadForShootingCommand(),
                        SHOOTING_CALCULATIONS::isReadyToShoot
                )
        );
    }

    private static Command getShootForDeliveryCommand() {
        return new ParallelCommandGroup(
                getAimForDeliveryCommand(),
                GeneralCommands.runWhen(
                        getLoadForShootingCommand(),
                        SHOOTING_CALCULATIONS::isReadyToShoot
                )
        );
    }

    public static Command getLoadForShootingCommand() {
        return new ParallelCommandGroup(
                IndexerCommands.getSetTargetStateCommand(IndexerConstants.IndexerState.LOAD_FOR_SHOOTING),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_SHOOTING)
        );
    }

    private static Command getAimAtHubCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        getTargetShootingAtHubLocationCommand(),
                        HoodCommands.getAimAtHubCommand(),
                        ShooterCommands.getAimAtHubCommand()
                )
        );
    }

    private static Command getTargetShootingAtHubLocationCommand() {
        return new InstantCommand(
                () -> SHOOTING_CALCULATIONS.setTargetShootingLocation(ShootingCalculations.TargetLocation.HUB)
        );
    }

    private static Command getAimForDeliveryCommand() {
        return new ParallelCommandGroup(
                getTargetShootingForDeliveryLocationCommand(),
                HoodCommands.getAimForDeliveryCommand(),
                ShooterCommands.getAimForDeliveryCommand()
        );
    }

    private static Command getTargetShootingForDeliveryLocationCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getTargetShootingForDeliveryRightLocationCommand(),
                getTargetShootingForDeliveryLeftLocationCommand(),
                () -> RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2
        );
    }

    private static Command getTargetShootingForDeliveryRightLocationCommand() {
        return new InstantCommand(
                () -> SHOOTING_CALCULATIONS.setTargetShootingLocation(ShootingCalculations.TargetLocation.RIGHT_DELIVERY_LOCATION)
        );
    }

    private static Command getTargetShootingForDeliveryLeftLocationCommand() {
        return new InstantCommand(
                () -> SHOOTING_CALCULATIONS.setTargetShootingLocation(ShootingCalculations.TargetLocation.LEFT_DELIVERY_LOCATION)
        );
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }
}