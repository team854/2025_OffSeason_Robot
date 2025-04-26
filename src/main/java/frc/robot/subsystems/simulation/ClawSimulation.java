package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.RectangularPrism;
import frc.robot.utilities.math.PoseUtilities;
import frc.robot.utilities.math.VectorUtilities;

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
    private double gamePieceOffset;

    private boolean active;

    private final String targetedGamePieceType;
    private final RectangularPrism pickupVolume;

    private Translation3d pickupVolumeVelocity = new Translation3d();
    private double lastVolumeUpdateTime = Timer.getFPGATimestamp();
    
    public ClawSimulation(RectangularPrism pickupVolume, int gamePieceCapacity) {
        this.targetedGamePieceType = "Coral";
        this.pickupVolume = pickupVolume;
        this.gamePieceCapacity = gamePieceCapacity;

        this.gamePieceCount = 0;
        this.active = false;

        this.gamePieceOffset = 0;
    }

    public void setActiveStage(boolean active) {
        this.active = active;
    }

    public void setGamePeiceCount(int count) {
        this.gamePieceCount = MathUtil.clamp(count, 0, this.gamePieceCapacity);
    }

    public void setPickupVolumeCenterPose(Pose3d centerPose) {

        double callTime = Timer.getFPGATimestamp();
        this.pickupVolumeVelocity = centerPose.getTranslation().minus(this.pickupVolume.getCenterPose().getTranslation()).div(callTime - this.lastVolumeUpdateTime);
        this.lastVolumeUpdateTime = callTime;

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

    public boolean ejectGamePiece() {
        // Make sure that there is a game piece to outtake
        if (this.gamePieceCount > 0) {
            Pose3d coralPose = getPiecePose();

            // Calculate the direction to shoot the coral at the specified speed
            Translation3d shootTranslation = new Translation3d(0, -Constants.ArmConstants.Intake.Simulation.OUTTAKE_VELOCITY.in(MetersPerSecond), 0).rotateBy(coralPose.getRotation());

            shootTranslation = shootTranslation.plus(this.pickupVolumeVelocity);

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new CoralFlightSim(ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                            coralPose.getTranslation(), shootTranslation,
                            coralPose.getRotation()));
            this.gamePieceCount--;
            return true;
        }
        return false;
    }

    /**
     * 
     * @return The pose of the piece when its being held by the claw including the coral offset
     */
    public Pose3d getPiecePose() {
        Pose3d clawPose = this.pickupVolume.getCenterPose();

        Pose3d coralPose = new Pose3d(clawPose.getTranslation(), new Rotation3d(clawPose.getRotation().getQuaternion().times(ninetyZRotation)));

        coralPose = coralPose.transformBy(new Transform3d(this.gamePieceOffset, 0, 0, new Rotation3d()));

        return coralPose;
    }

    /**
     * Checks if the piece and the claw are aligned (Credit to claude 3.7)(I only asked it for the dot product cause I didn't know about the cross product (oops))
     * 
     * @param piecePose The pose of the game piece
     * @param clawQuaternion The quaternion of the claw rotated by 90 degrees in the z axis
     * @return If the two are aligned
     */
    private boolean checkAlignment(Pose3d piecePose, Translation3d clawNinety) {
        Quaternion pieceQuaternion = piecePose.getRotation().getQuaternion();

        Translation3d pieceForward = new Translation3d(1, 0, 0).rotateBy(new Rotation3d(pieceQuaternion.toRotationVector()));

        // Compute the dot product of both translation vectors
        // Using the quaternion to compute the dot product doesn't product the expected result
        double rotationNormalizedDotProduct = VectorUtilities.normalizedDotProduct(clawNinety, pieceForward);

        // The dot product is 1 or -1 if they are perfectly aligned
        // 1 - the absolute of the dot product is how close it is to 1 or -1
        return rotationNormalizedDotProduct < 0.125;
    }

    public Distance getCoralOffset() {
        return Meter.of(this.gamePieceOffset);
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
        Translation3d clawNinety = new Translation3d(1, 0, 0).rotateBy(getPiecePose().getRotation());

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
                // Calculate how far the center of the coral is from the center of the pickup volume
                double pieceDistance = PoseUtilities.calculatePoseDistance(gamePiecePose, clawPose).in(Meter);

                if (pieceDistance < lowestDistance) {

                    // Check if the claw and the piece are aligned with eachother
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
                // Calculate how far the center of the coral is from the center of the pickup volume
                double pieceDistance = PoseUtilities.calculatePoseDistance(gamePiecePose, clawPose).in(Meter);

                if (pieceDistance < lowestDistance) {

                    // Check if the claw and the piece are aligned with eachother
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

        // Calculate the offset of the piece relative to the claw
        Translation3d pieceTranslation = lowestPiece.piecePose.getTranslation();

        // Calculate a pose for the back of the claw
        Pose3d backClawPose = clawPose.transformBy(new Transform3d(-0.08, 0, 0, new Rotation3d()));

        // Calcualte the differnce bettween the back of the claw and the piece independant of rotation
        Translation3d rawPieceDiff = backClawPose.getTranslation().minus(pieceTranslation);

        // Calculate the difference in the required angle to point to the piece andthe angle of the claw
        double localAngleToPiece = Math.atan2(rawPieceDiff.getZ(), Math.hypot(rawPieceDiff.getX(), rawPieceDiff.getY())) - backClawPose.getRotation().getY();

        // If its too high its unlikly that the piece would acully be able to be picked up
        if (Math.abs(localAngleToPiece) > Units.degreesToRadians(30)) {
            return;
        }

        // Calcualte the differnce bettween the claw and the piece while taking into account rotation
        Translation3d pieceDiff = pieceTranslation.minus(clawPose.getTranslation()).rotateBy(clawPose.getRotation().unaryMinus());

        this.gamePieceOffset = pieceDiff.getY();

        if (lowestPiece.objectReference instanceof GamePieceOnFieldSimulation) {
            // If its a field piece it remove notify it that it is being intaked then remove it
            GamePieceOnFieldSimulation gamePiece = (GamePieceOnFieldSimulation) lowestPiece.objectReference;

            if (gamePiece.type == "Coral") {
                gamePiece.onIntake(this.targetedGamePieceType);
                arena.removeGamePiece(gamePiece);
                this.gamePieceCount++;
            }
            

        } else if (lowestPiece.objectReference instanceof GamePieceProjectile) {
            // If its a projectile just remove it from the field
            GamePieceProjectile gamePiece = (GamePieceProjectile) lowestPiece.objectReference;

            if (gamePiece.gamePieceType == "Coral") {
                arena.removeProjectile(gamePiece);
                this.gamePieceCount++;
            }
        }
    }


}
