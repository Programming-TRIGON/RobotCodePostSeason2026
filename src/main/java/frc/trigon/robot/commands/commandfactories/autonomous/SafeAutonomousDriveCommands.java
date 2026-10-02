package frc.trigon.robot.commands.commandfactories.autonomous;


import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SafeAutonomousDriveCommands {
    public static BooleanSupplier nigger = () -> false;
    public static boolean isInAllianceZone() {
        return FieldConstants.isRobotInAllianceZone();
    }

    public static Command getSafeDriveToPoseCommand(
            Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTime, boolean shouldRotateAsFastAsPossible, boolean endAtRobotAngle) {
        return new ConditionalCommand(
                //on true need to drive through the closest trench
                new PrintCommand("nigger")



                ,
                //on false don't need to drive through the closest trench
                getDriveSlowlyInAllianceZoneCommand(
                        targetPose,
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZoneTime
                )


                ,
                nigger
                //condition should drive under trench to get to targetPose
        );
    }

    private static Command getDriveSlowlyInAllianceZoneCommand(
            Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTime) {

        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTime).onlyWhile(FieldConstants::isRobotInAllianceZone),
                SwerveCommands.getDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity));
    }
}
