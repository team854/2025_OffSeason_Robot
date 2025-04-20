package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.RectangularPrism;
import frc.robot.objects.VisionEstimate;
import frc.robot.utilities.field.TagUtilities;

public class SimulationSubsystem extends SubsystemBase {
    private CoralStationSimulation[] coralStationArray;

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

        // If they are enabled setup the coral stations
        if (Constants.SimulationConstants.CoralStations.ENABLED) {
            setupCoralStations();
        }
    }

    private void spawnCoralInRadius() {

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

    private void setupCoralStations() {

        List<CoralStationSimulation> tempCoralStations = new ArrayList<>();

        int[] coralStationTagIDs = RobotContainer.isBlueAlliance()
                ? Constants.CoralStationConstants.FieldConstants.BLUE_ALLIANCE_CORAL_STATION_TAG_IDS
                : Constants.CoralStationConstants.FieldConstants.RED_ALLIANCE_CORAL_STATION_TAG_IDS;

        for (int tagID : coralStationTagIDs) {
            // Get the pose of the tag
            Pose3d tagPose = TagUtilities.getTagPose(tagID, true);

            // Offset the pose of the tag to get the pose of the coral station
            Pose3d coralStationPose = tagPose
                    .transformBy(new Transform3d(Constants.SimulationConstants.CoralStations.FOWARD_OFFSET.in(Meter), 0,
                            Constants.SimulationConstants.CoralStations.VERTICAL_OFFSET.in(Meter), new Rotation3d()));

            // Offset the pose of the tag to get the pose that coral should be spawned at
            Pose3d coralSpawnPose = tagPose.transformBy(
                    new Transform3d(0, 0, Constants.SimulationConstants.CoralStations.VERTICAL_OFFSET.in(Meter),
                            new Rotation3d(0, Constants.SimulationConstants.CoralStations.RAMP_ANGLE.in(Radian), Units.degreesToRadians(180))));

            // Create the coral station simulation and add it to the temp list
            CoralStationSimulation coralStation = new CoralStationSimulation(coralStationPose, coralSpawnPose);
            tempCoralStations.add(coralStation);
        }

        // Move the temp list to the array
        coralStationArray = tempCoralStations.toArray(new CoralStationSimulation[0]);
    }

    @Override
    public void simulationPeriodic() {
        if (Constants.SimulationConstants.CoralStations.ENABLED) {
            for (CoralStationSimulation coralStation : coralStationArray) {
                coralStation.iterate();
            }
        }
    }
}
