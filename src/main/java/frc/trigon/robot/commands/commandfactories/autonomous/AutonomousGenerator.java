package frc.trigon.robot.commands.commandfactories.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousGenerator {
    public static final LoggedDashboardChooser<AutonomousState>
            FIRST_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FirstAutonomousChooser", new SendableChooser<>()),
            SECOND_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SecondAutonomousChooser", new SendableChooser<>()),
            THIRD_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("ThirdAutonomousChooser", new SendableChooser<>()),
            FOURTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FourthAutonomousChooser", new SendableChooser<>()),
            FIFTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("FifthAutonomousChooser", new SendableChooser<>()),
            SIXTH_AUTONOMOUS_CHOOSER = new LoggedDashboardChooser<>("SixthAutonomousChooser", new SendableChooser<>());

/*
    public static Command getAutonomousCommand() {
    TODO: implement autonomous commands
    }
*/

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

    enum AutonomousState {
        SCORE,
        DELIVERY,
        COLLECT_FROM_NATURAL_ZONE;

        AutonomousState() {
        }
    }
}
