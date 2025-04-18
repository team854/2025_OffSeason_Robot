package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.objects.VisionEstimate;

public class VisionCamera {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraOffset;
    private final PhotonPoseEstimator photonPoseEstimator;

    /*
     * Constrained solve params
     */
    private final Optional<ConstrainedSolvepnpParams> solveParams = Optional
            .of(new ConstrainedSolvepnpParams(true, 0.5));

    /*
     * Camera properties
     */
    private final double effectiveRange;

    /*
     * Simulation
     */
    private PhotonCameraSim cameraSim;

    public VisionCamera(String cameraName, Transform3d cameraOffset, int width, int height, int fps,
            double diagonal_fov, double average_pixel_error, double average_pixel_error_std_devs,
            double average_latency, double average_latency_std_devs, double effectiveRange, VisionSystemSim visionSim) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraOffset = cameraOffset;
        this.effectiveRange = effectiveRange;


        System.out.println("cameraOffset " + cameraOffset.toString());

        photonPoseEstimator = new PhotonPoseEstimator(Constants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cameraOffset);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

        // If its a simulation setup the simulated camera
        if (Robot.isSimulation()) {

            // Setup the virual camera with properties mesured from the real camera
            SimCameraProperties cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(width, height, Rotation2d.fromDegrees(diagonal_fov));
            cameraProperties.setCalibError(average_pixel_error, average_pixel_error_std_devs);
            cameraProperties.setFPS(fps);
            cameraProperties.setAvgLatencyMs(average_latency);
            cameraProperties.setLatencyStdDevMs(average_latency_std_devs);

            this.cameraSim = new PhotonCameraSim(this.camera, cameraProperties);

            this.cameraSim.setMaxSightRange(effectiveRange);
            this.cameraSim.setMinTargetAreaPixels(35);
            this.cameraSim.enableDrawWireframe(true);

            visionSim.addCamera(this.cameraSim, this.cameraOffset);
        }

    }

    public String getCameraName() {
        return this.cameraName;
    }

    public Distance getEffectiveRange() {
        return Meter.of(this.effectiveRange);
    }

    /**
     * 
     * @return The pose of the camera in global space
     */
    public Pose3d getCameraPose() {
        Pose3d swervePose = new Pose3d(RobotContainer.swerveSubsystem.getPose()).plus(
                new Transform3d(0, 0, Constants.RobotKinematicConstants.HEIGHT_OFF_GROUND.in(Meter), new Rotation3d()));

        return swervePose.plus(this.cameraOffset);
    }
    
    public Optional<Pose3d> getTagPose(int tagID) {
        return photonPoseEstimator.getFieldTags().getTagPose(tagID);
    }

    public Optional<VisionEstimate> getLatestEstimate() {
        // Update the pose estimator with the current heading
        // Methods like PNP_DISTANCE_TRIG_SOLVE need this
        this.photonPoseEstimator.addHeadingData(Timer.getFPGATimestamp(),
                RobotContainer.swerveSubsystem.swerveDrive.getGyroRotation3d());

        // Get all results and figure out which
        // estimate is the most recent
        List<PhotonPipelineResult> photonResults = camera.getAllUnreadResults();
        double latestTiemstamp = -1;
        VisionEstimate latestPose = null;

        for (PhotonPipelineResult result : photonResults) {

            Optional<EstimatedRobotPose> tempRobotEstimate = photonPoseEstimator.update(result, Optional.empty(),
                    Optional.empty(), this.solveParams);

            if (tempRobotEstimate.isEmpty()) {
                continue;
            }

            VisionEstimate foundRobotEstimate = new VisionEstimate(tempRobotEstimate.get());

            // Update the most recent estimated pose
            if (foundRobotEstimate.timestampSeconds > latestTiemstamp) {
                latestTiemstamp = foundRobotEstimate.timestampSeconds;
                latestPose = foundRobotEstimate;
            }

        }

        return Optional.ofNullable(latestPose);
    }
}
