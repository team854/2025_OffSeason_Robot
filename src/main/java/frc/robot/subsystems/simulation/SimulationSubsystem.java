package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meter;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.RectangularPrism;
import frc.robot.objects.VisionEstimate;
import frc.robot.utilities.field.TagUtilities;

public class SimulationSubsystem extends SubsystemBase{
    private RectangularPrism[] coralStationArray;

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
        
        List<RectangularPrism> tempCoralStations = new ArrayList<>();

        int[] coralStationTagIDs = RobotContainer.isBlueAlliance()
                ? Constants.CoralStationConstants.FieldConstants.BLUE_ALLIANCE_CORAL_STATION_TAG_IDS
                : Constants.CoralStationConstants.FieldConstants.RED_ALLIANCE_CORAL_STATION_TAG_IDS;

        for (int tagID : coralStationTagIDs) {
            // Get the pose of the tag
            Pose3d tagPose = TagUtilities.getTagPose(tagID, true);

            // Offset the pose of the tag to get the pose of the coral station
            Pose3d coralStationPose = tagPose.transformBy(new Transform3d(Constants.SimulationConstants.CoralStations.FOWARD_OFFSET.in(Meter), 0, Constants.SimulationConstants.CoralStations.VERTICAL_OFFSET.in(Meter), new Rotation3d()));

            // Create the rectangular prism and add it to the temp list
            RectangularPrism coralStation = new RectangularPrism(coralStationPose, Constants.SimulationConstants.CoralStations.WIDTH, Constants.SimulationConstants.CoralStations.LENGTH, Constants.SimulationConstants.CoralStations.HEIGHT);
            tempCoralStations.add(coralStation);
        }

        // Move the temp list to the array
        coralStationArray = tempCoralStations.toArray(new RectangularPrism[0]);
    }

    private void checkCoralStationClawOverlap() {
        RectangularPrism clawVolume = RobotContainer.endEffectorSubsystem.getClawSimulation().getPickupVolume();
        for (RectangularPrism coralStation : coralStationArray) {
            boolean overlap = coralStation.checkOverlap(clawVolume);

            SmartDashboard.putBoolean("OVERLAP", overlap);
            SmartDashboard.putNumberArray("CORALSTATION", coralStation.generateDebugWireframe());
            break;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (Constants.SimulationConstants.CoralStations.ENABLED) {
            checkCoralStationClawOverlap();
        }
    }
}
