package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousGenerator {
    public static final LoggedDashboardChooser<AutonomousState>
            FIRST_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FirstAutonomousChooser", new SendableChooser<>()),
            SECOND_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SecondAutonomousChooser", new SendableChooser<>()),
            THIRD_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("ThirdAutonomousChooser", new SendableChooser<>()),
            FOURTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FourthAutonomousChooser", new SendableChooser<>()),
            FIFTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FifthAutonomousChooser", new SendableChooser<>()),
            SIXTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SixthAutonomousChooser", new SendableChooser<>());

    public static void init() {
        configAutonomousChooser(FIRST_AUTONOMOUS_CHOOSER);
        configAutonomousChooser(SECOND_AUTONOMOUS_CHOOSER);
        configAutonomousChooser(THIRD_AUTONOMOUS_CHOOSER);
        configAutonomousChooser(FOURTH_AUTONOMOUS_CHOOSER);
        configAutonomousChooser(FIFTH_AUTONOMOUS_CHOOSER);
        configAutonomousChooser(SIXTH_AUTONOMOUS_CHOOSER);
    }

    public static void configAutonomousChooser(LoggedDashboardChooser<AutonomousState> chooser) {
        chooser.addOption("Score", AutonomousState.SCORE);
        chooser.addOption("Delivery", AutonomousState.DELIVERY);
        chooser.addOption("CollectFromNaturalZone", AutonomousState.COLLECT_FROM_NATURAL_ZONE);
        chooser.addOption("Nothing", null);
    }

    public static Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                getAutonomousStateSequenceCommand(),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)
        );
    }

    private static Command getAutonomousStateSequenceCommand() {
        return new SequentialCommandGroup(
                getCommandFromState(0),
                getCommandFromState(1),
                getCommandFromState(2),
                getCommandFromState(3),
                getCommandFromState(4),
                getCommandFromState(5)
        );
    }


    private static Command getCommandFromState(int index) {
        AutonomousState state = getStateFromIndex(index);

        if (state == null)
            return Commands.none();

        return switch (state) {
            case SCORE -> GeneralAutonomousCommands.getAutonomousScoreCommand();
            case DELIVERY -> GeneralAutonomousCommands.getAutonomousDeliveryCommand();
            case COLLECT_FROM_NATURAL_ZONE -> Commands.none();
        };

    }

    private static AutonomousState getStateFromIndex(int index) {
        return switch (index) {
            case 0 -> FIRST_AUTONOMOUS_CHOOSER.get();
            case 1 -> SECOND_AUTONOMOUS_CHOOSER.get();
            case 2 -> THIRD_AUTONOMOUS_CHOOSER.get();
            case 3 -> FOURTH_AUTONOMOUS_CHOOSER.get();
            case 4 -> FIFTH_AUTONOMOUS_CHOOSER.get();
            case 5 -> SIXTH_AUTONOMOUS_CHOOSER.get();
            default -> null;
        };
    }

    enum AutonomousState {
        SCORE,
        DELIVERY,
        COLLECT_FROM_NATURAL_ZONE;

        AutonomousState() {
        }
    }
}
