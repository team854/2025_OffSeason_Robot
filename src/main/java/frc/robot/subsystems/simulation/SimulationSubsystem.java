package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        // Spawn the coral in a radius
        if (Constants.SimulationConstants.StartingSpawnCoral.ENABLED) {
            spawnCoralInRadius();
        }
    }

    public void spawnCoralInRadius() {
        
        // Get a pose for the middle of the spawn area
        Pose2d middlePose = new Pose2d(
                    new Translation2d(Constants.SimulationConstants.StartingSpawnCoral.SPAWN_X.in(Meter),
                            Constants.SimulationConstants.StartingSpawnCoral.SPAWN_Y.in(Meter)),
                    Rotation2d.fromDegrees(Math.random() * 360));

        for (int i = 0; i < Constants.SimulationConstants.StartingSpawnCoral.SPAWN_COUNT; i++) {

            // Offset the spawnpoint by a random distance and angle
            Pose2d spawnPose = middlePose.plus(new Transform2d(
                    Math.random()
                            * Constants.SimulationConstants.StartingSpawnCoral.SPAWN_RADIUS.in(Meter),
                    0, new Rotation2d()));

            // At the game piece at a random angle
            SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                    new Pose2d(spawnPose.getX(), spawnPose.getY(), Rotation2d.fromDegrees(Math.random() * 360))));
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
