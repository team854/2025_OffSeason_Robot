package frc.robot.objects;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * 
 * VisionEstimate is bassicly a copy of {@link EstimatedRobotPose} with the ablity to store std devs
 * 
 */
public class VisionEstimate {
    // The estimated pose
    public Pose3d estimatedPose;

    // The estimated time the frame used to derive the robot pose was taken
    public double timestampSeconds;

    // A list of the targets used to compute the pose
    public List<PhotonTrackedTarget> targetsUsed;

    // The std devs of the pose
    public Matrix<N3, N1> stdDevs;

    // The strategy actually used to produce the pose
    public PoseStrategy strategy;

    /**
     * Constructs an VisionEstimate
     *
     * @param estimatedPose estimated pose
     * @param timestampSeconds timestamp of the estimate
     */
    public VisionEstimate(
            Pose3d estimatedPose,
            double timestampSeconds,
            List<PhotonTrackedTarget> targetsUsed,
            Matrix<N3, N1> stdDevs,
            PoseStrategy strategy) {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.stdDevs = stdDevs;
        this.strategy = strategy;
    }

    /**
     * Constructs an VisionEstimate using a {@link EstimatedRobotPose}
     *
     * @param estimatedRobotPose The {@link EstimatedRobotPose} to be translated to a VisionEstimate
     */
    public VisionEstimate(EstimatedRobotPose estimatedRobotPose) {
        this(estimatedRobotPose.estimatedPose,
            estimatedRobotPose.timestampSeconds,
            estimatedRobotPose.targetsUsed,
            VecBuilder.fill(0,0,0),
            estimatedRobotPose.strategy);
    }
}
