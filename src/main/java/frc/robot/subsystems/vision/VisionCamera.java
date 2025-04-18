package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Robot;

public class VisionCamera {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraOffset;
    private final PhotonPoseEstimator photonPoseEstimator;

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

        photonPoseEstimator = new PhotonPoseEstimator(Constants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cameraOffset);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

		// If its a simulation setup the simulated camera
        if (Robot.isSimulation()) {
			SimCameraProperties cameraProperties = new SimCameraProperties();
        }

        FINISH ME

    }
}
