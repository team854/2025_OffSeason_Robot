package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Vector;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.RectangularPrism;
import frc.robot.utilities.math.PoseUtilities;
import frc.robot.utilities.math.VectorUtilities;

public class CoralStationSimulation {
    private final RectangularPrism coralStationVolume;
    private final Pose3d coralSpawnPose;
    private final Translation3d launchVector;
    private final RectangularPrism clawVolume;

    private int inZoneCount = 0;
    private Timer inZoneTimer = new Timer();

    public CoralStationSimulation(Pose3d centerPose, Pose3d coralSpawnPose) {
        this.coralStationVolume = new RectangularPrism(centerPose, Constants.SimulationConstants.CoralStations.WIDTH, Constants.SimulationConstants.CoralStations.LENGTH, Constants.SimulationConstants.CoralStations.HEIGHT);
        this.coralSpawnPose = coralSpawnPose;

        // Reset timer and stop it
        this.inZoneCount = 0;
        this.inZoneTimer.reset();
        this.inZoneTimer.stop();

        // Get the claw volume in the simulated claw
        this.clawVolume = RobotContainer.endEffectorSubsystem.getClawSimulation().getPickupVolume();

        // Create a vector with the inverse of direction that the coral will be launched
        this.launchVector = new Translation3d(-1, 0, 0).rotateBy(coralSpawnPose.getRotation());
    }

    private Pose3d offsetCoralSpawnPoint(Pose3d endEffPose) {
        Translation3d endEffTranslation = endEffPose.getTranslation();

        endEffTranslation = endEffTranslation.minus(this.coralSpawnPose.getTranslation()).rotateBy(this.coralSpawnPose.getRotation().unaryMinus());

        return this.coralSpawnPose.transformBy(new Transform3d(0, endEffTranslation.getY(), 0, new Rotation3d()));
    }

    private void spawnRampCoral(Pose3d coralSpawnPose) {
        // Calculate the spawn roation of the coral so it spawns horizontal
        Rotation3d spawnRotation = coralSpawnPose.getRotation();
        spawnRotation = new Rotation3d(spawnRotation.getY(), 0, spawnRotation.getZ() + Units.degreesToRadians(90));

        // Calculate the direction to shoot the coral at the specified speed
        Translation3d shootTranslation = new Translation3d(0, -Constants.SimulationConstants.CoralStations.SPAWN_VELOCITY.in(MetersPerSecond), 0).rotateBy(spawnRotation);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new CoralFlightSim(ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                        coralSpawnPose.getTranslation(), shootTranslation,
                        spawnRotation));
    }

    public RectangularPrism getSpawnVolume() {
        return this.coralStationVolume;
    }

    public void iterate() {
        boolean overlap = coralStationVolume.checkOverlap(this.clawVolume);

        // Get intake percent
        double intakePercent = RobotContainer.endEffectorSubsystem.getIntakeSpeedPercent();

        if (!overlap || intakePercent > -0.2) {
            this.inZoneCount = 0;
            this.inZoneTimer.reset();
            this.inZoneTimer.stop();
            return;
        }

        // Get the pose of the robot from the swerve simulation
        Pose3d robotPose = new Pose3d(RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get());

        // Get a normalized vector for the end of the end effector
        Pose3d endEffPose = RobotContainer.endEffectorSubsystem.calculateEndEffectorPose(robotPose);
        Translation3d endEffRotationalVector = new Translation3d(1, 0, 0).rotateBy(endEffPose.getRotation());

        // Get the difference bettween the inverse of the launch vector and the end effector vector
        double directionalDiff = VectorUtilities.normalizedDotProduct(endEffRotationalVector, this.launchVector);

        // Get the rotation of the wrist
        double wristRotation = Math.min(Math.abs(RobotContainer.wristSubsystem.getWristAngle().in(Degree)) % 180, 180 - (Math.abs(RobotContainer.wristSubsystem.getWristAngle().in(Degree)) % 180));
    
        if (directionalDiff > Constants.SimulationConstants.CoralStations.ANGLE_DIFFERENCE_THRESHOLD
                 || wristRotation > Constants.SimulationConstants.CoralStations.WRIST_DIFFERENCE_THRESHOLD.in(Degree)) {
            this.inZoneCount = 0;
            this.inZoneTimer.reset();
            this.inZoneTimer.stop();
            return;
        }

        if (inZoneTimer.isRunning()) {

            // Check to make sure a coral hasn't already been dispensed and it has been a certain number of seconds
            if (this.inZoneCount == 1 && inZoneTimer.hasElapsed(Constants.SimulationConstants.CoralStations.IN_ZONE_TIME.in(Second))) {
                Pose3d finalCoralSpawnPose = offsetCoralSpawnPoint(endEffPose);

                System.out.println("Dispensing coral at " + finalCoralSpawnPose);

                spawnRampCoral(finalCoralSpawnPose);
                this.inZoneCount = 2;
            }

        } else {
            this.inZoneCount = 1;
            this.inZoneTimer.reset();
            this.inZoneTimer.start();
        }
    }
}
