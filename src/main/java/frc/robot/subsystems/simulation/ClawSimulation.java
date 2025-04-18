package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.objects.RectangularPrism;
import frc.robot.utilities.math.PoseUtilities;

public class ClawSimulation {

    /*
     * Constants
     */
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final Quaternion ninetyZRotation = new Quaternion(
            Math.cos(Units.degreesToRadians(90) / 2),
            0,
            0,
            Math.sin(Units.degreesToRadians(90) / 2)
        ); // Thanks claude 3.7 xD

    private final int gamePieceCapacity;
    private int gamePieceCount;

    private boolean active;

    private final String targetedGamePieceType;
    private final RectangularPrism pickupVolume;
    
    public ClawSimulation(RectangularPrism pickupVolume, int gamePieceCapacity) {
        this.targetedGamePieceType = "Coral";
        this.pickupVolume = pickupVolume;
        this.gamePieceCapacity = gamePieceCapacity;

        this.gamePieceCount = 0;
        this.active = false;
    }

    public void setActiveStage(boolean active) {
        this.active = active;
    }

    public void setGamePeiceCount(int count) {
        this.gamePieceCount = MathUtil.clamp(count, 0, this.gamePieceCapacity);
    }

    public void setPickupVolumeCenterPose(Pose3d centerPose) {
        this.pickupVolume.changeCenterPose(centerPose);
    }

    public Pose3d getPickupVolumeCenterPose() {
        return this.pickupVolume.getCenterPose();
    }

    public RectangularPrism getPickupVolume() {
        return this.pickupVolume;
    }

    public int getGamePieceCount() {
        return this.gamePieceCount;
    }

    private Translation3d getCoralShootDirection(Pose3d coralPose, double speed) {
        Translation3d shootTranslation = new Translation3d(0, -speed, 0).rotateBy(coralPose.getRotation());
        return shootTranslation;
    }

    public void ejectGamePiece() {
        if (this.gamePieceCount > 0) {
            Pose3d robotPose = new Pose3d(RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get());

            Pose3d coralPose = RobotContainer.endEffectorSubsystem.calculateCoralPose(robotPose);
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new CoralFlightSim(ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                            coralPose.getTranslation(), getCoralShootDirection(coralPose, 0.3),
                            coralPose.getRotation()));
            this.gamePieceCount--;
        }
    }

    /**
     * Checks if the piece and the claw are aligned (Credit to claude 3.7)
     * 
     * @param piecePose The pose of the game piece
     * @param clawQuaternion The quaternion of the claw rotated by 90 degrees in the z axis
     * @return If the two are aligned
     */
    private boolean checkAlignment(Pose3d piecePose, Translation3d clawNinety) {
        Quaternion pieceQuaternion = piecePose.getRotation().getQuaternion();

        Translation3d pieceForward = new Translation3d(1, 0, 0).rotateBy(new Rotation3d(pieceQuaternion.toRotationVector()));
        double rotationDotProduct = (clawNinety.getX() * pieceForward.getX()) + 
                                    (clawNinety.getY() * pieceForward.getY()) + 
                                    (clawNinety.getZ() * pieceForward.getZ());

        return 1 - Math.abs(rotationDotProduct) < 0.1;
    }

    private record PickupCandidate(
        Pose3d piecePose,
        Object objectReference
    ) {}

    public void iterate() {
        if (!this.active) {
            return;
        }

        if (this.gamePieceCount >= this.gamePieceCapacity) {
            return;
        }

        Pose3d clawPose = this.pickupVolume.getCenterPose();
        Rotation3d clawRotation = clawPose.getRotation();
        Quaternion clawQuaternion = clawRotation.getQuaternion().times(ninetyZRotation);
        Translation3d clawNinety = new Translation3d(1, 0, 0).rotateBy(new Rotation3d(clawQuaternion.toRotationVector()));

        // Closest pickup candidate
        double lowestDistance = Double.MAX_VALUE;
        PickupCandidate lowestPiece = null;

        // Handle pieces on the field
        for (GamePieceOnFieldSimulation gamePiece : arena.gamePiecesOnField()) {
            if (!Objects.equals(gamePiece.type, this.targetedGamePieceType)) {
                continue;
            }

            Pose3d gamePiecePose = gamePiece.getPose3d();
            if (this.pickupVolume.poseInside(gamePiecePose)) {
                double pieceDistance = PoseUtilities.calculatePoseDistance(gamePiecePose, clawPose).in(Meter);

                if (pieceDistance < lowestDistance) {
                    if (checkAlignment(gamePiecePose, clawNinety)) {
                        lowestDistance = pieceDistance;
                        lowestPiece = new PickupCandidate(gamePiecePose, gamePiece);
                    }
                }
            }
        }

        // Handle pieces in the air
        for (GamePieceProjectile gamePiece : arena.gamePieceLaunched()) {
            if (!Objects.equals(gamePiece.gamePieceType, this.targetedGamePieceType)) {
                continue;
            }

            Pose3d gamePiecePose = gamePiece.getPose3d();
            if (this.pickupVolume.poseInside(gamePiecePose)) {
                double pieceDistance = PoseUtilities.calculatePoseDistance(gamePiecePose, clawPose).in(Meter);

                if (pieceDistance < lowestDistance) {
                    if (checkAlignment(gamePiecePose, clawNinety)) {
                        lowestDistance = pieceDistance;
                        lowestPiece = new PickupCandidate(gamePiecePose, gamePiece);
                    }
                }
            }
        }

        if (lowestPiece == null) {
            return;
        }

        if (lowestPiece.objectReference instanceof GamePieceOnFieldSimulation) {
            GamePieceOnFieldSimulation gamePiece = (GamePieceOnFieldSimulation) lowestPiece.objectReference;
            gamePiece.onIntake(this.targetedGamePieceType);
            arena.removeGamePiece(gamePiece);
            this.gamePieceCount++;

        } else if (lowestPiece.objectReference instanceof GamePieceProjectile) {
            GamePieceProjectile gamePiece = (GamePieceProjectile) lowestPiece.objectReference;
            arena.removeProjectile(gamePiece);
            this.gamePieceCount++;
        }
    }
}
