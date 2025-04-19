package frc.robot.subsystems.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SimulationSubsystem extends SubsystemBase{


    public SimulationSubsystem() {
        SimulatedArena.getInstance().resetFieldForAuto();

        // Check if the bot should start with a coral
        if (Constants.SimulationConstants.ASSUME_START_WITH_CORAL) {
            System.out.println("Starting robot with coral");
            // Set the ammount of coral in the claw to 1
            RobotContainer.endEffectorSubsystem.getClawSimulation().setGamePeiceCount(1);
        }  
    }
}
