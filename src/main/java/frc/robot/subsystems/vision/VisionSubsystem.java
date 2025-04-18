package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.objects.VisionEstimate;
import frc.robot.utilities.field.TagUtilities;
import frc.robot.utilities.files.JsonUtilities;

public class VisionSubsystem extends SubsystemBase {
    public VisionCamera[] visionCameras;
	public VisionEstimate[] visionEstimates;

	/*
	 * Simulation
	 */
    public VisionSystemSim visionSim;
    private boolean isSimulation = Robot.isSimulation();

    public VisionSubsystem() {
		// Setup the simulation and give it the april tag layout
		if (isSimulation) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.APRIL_TAG_FIELD_LAYOUT);
        }

		boolean anyCameras = setupCameras();

        if (!anyCameras) {
            System.out.println("No cameras configured");;
        }
    }

    private boolean setupCameras() {
		List<VisionCamera> tempCameras = new ArrayList<>();
		List<VisionEstimate> tempEstimates = new ArrayList<>();

        for (Class<?> limelightConfig : Constants.VisionConstants.class.getDeclaredClasses()) {
            String limelightName;
            try {
                limelightName = (String) limelightConfig.getDeclaredField("NAME").get(null);
                if ((boolean) limelightConfig.getDeclaredField("ENABLED").get(null) == true) {

                    Translation3d limelightRelativePosition = new Translation3d(
                            ((Distance) limelightConfig.getDeclaredField("FRONT_OFFSET").get(null)).in(Meter),
                            ((Distance) limelightConfig.getDeclaredField("LEFT_OFFSET").get(null)).in(Meter),
                            ((Distance) limelightConfig.getDeclaredField("HEIGHT_OFFSET").get(null)).in(Meter));

                    Rotation3d limelightRelativeRotation = new Rotation3d(
                            ((Angle) limelightConfig.getDeclaredField("ROLL").get(null)).in(Radian),
                            ((Angle) limelightConfig.getDeclaredField("PITCH").get(null)).in(Radian),
                            ((Angle) limelightConfig.getDeclaredField("YAW").get(null)).in(Radian));

                    double effectiveRange = ((Distance) limelightConfig.getDeclaredField("EFFECTIVE_RANGE").get(null)).in(Meter);

                    Class<?> cameraProperties = JsonUtilities.getInnerClass(limelightConfig, "CameraProperties");

                    int width = (int) cameraProperties.getDeclaredField("WIDTH").get(null);
                    int height = (int) cameraProperties.getDeclaredField("HEIGHT").get(null);
                    int fps = (int) cameraProperties.getDeclaredField("FPS").get(null);
                    double diagonal_fov = ((Angle) cameraProperties.getDeclaredField("DIAGONAL_FOV").get(null)).in(Degree);

                    double average_pixel_error = (double) cameraProperties.getDeclaredField("AVERGAGE_PIXEL_ERROR")
                            .get(null);
                    double average_pixel_error_std_devs = (double) cameraProperties
                            .getDeclaredField("AVERGAGE_PIXEL_ERROR_STD_DEVS").get(null);

                    double average_latency = (double) cameraProperties.getDeclaredField("AVERAGE_LATENCY").get(null);
                    double average_latency_std_devs = (double) cameraProperties
                            .getDeclaredField("AVERAGE_LATENCY_STD_DEVS").get(null);

					// Populate the lists
					tempCameras.add(new VisionCamera(limelightName,
                            new Transform3d(limelightRelativePosition, limelightRelativeRotation), width, height, fps,
                            diagonal_fov, average_pixel_error, average_pixel_error_std_devs, average_latency,
                            average_latency_std_devs, effectiveRange, this.visionSim));
					tempEstimates.add(new VisionEstimate(new Pose3d(), 0.0, null, VecBuilder.fill(0,0,0), null));

                    System.out.println("Camera with name " + limelightName + " is enabled");
                } else {
                    System.out.println("Camera with name " + limelightName + " is not enabled");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
		// Copy the temp list to the arrays
		visionCameras = tempCameras.toArray(new VisionCamera[0]);
		visionEstimates = tempEstimates.toArray(new VisionEstimate[0]);

        return tempCameras.size() > 0;
    }

	/**
	 * 
	 * @param visionEstimate The {@link VisionEstimate} to update the std devs of
	 * @param camera The {@link VisionCamera} that produced the {@link VisionEstimate}
	 * @return A matrix containing the std devs
	 */
	private Matrix<N3, N1> updateEstimateStdDevs(VisionEstimate visionEstimate, VisionCamera camera) {
		Matrix<N3, N1> stdDevs = Constants.VisionConstants.VISION_SINGLE_TAG_STD_DEVS;
        int validTargets = 0;
        double averageDistance, averageAmbiguity;
        averageDistance = averageAmbiguity = 0;

		// Get the translation of the camera in global space
		Translation3d cameraTranslation = camera.getCameraPose().getTranslation();

        for (var target : visionEstimate.targetsUsed) {
            Optional<Pose3d> targetPose = camera.getTagPose(target.getFiducialId());

			// Make sure that it acully returned a pose
            if (targetPose.isEmpty()) {
                continue;
            }

            validTargets++;
            averageAmbiguity += target.getPoseAmbiguity();
            averageDistance += targetPose
					.get()
                    .getTranslation()
                    .getDistance(cameraTranslation);
        }

		// Average out the distance and ambiguity
        averageDistance /= validTargets;
        averageAmbiguity /= validTargets;

        if (validTargets > 1) {
            stdDevs = Constants.VisionConstants.VISION_MULTI_TAG_STD_DEVS;
        }

        if (validTargets == 1 && averageDistance > camera.getEffectiveRange().in(Meter)) {
            stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            stdDevs = stdDevs.times(
                    1 + (Math.pow(averageDistance, 2) / Constants.VisionConstants.TARGET_DISTANCE_STD_DEVS_DIVISOR)
                            + (averageAmbiguity / Constants.VisionConstants.TARGET_AMBIGUITY_STD_DEVS_DIVISOR));
        }

		return stdDevs;
	}

    public void updateVisionEstimates() {
        if (isSimulation) {
            // Update the vision simulation with the exact robot pose from the drive train
            // simulation
            RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().ifPresent(pose -> {
                visionSim.update(pose);
            });
        }

        for (int index = 0; index < visionCameras.length; index++) {
			VisionCamera camera = visionCameras[index];
            
			Optional<VisionEstimate> tempEstimatedPose = camera.getLatestEstimate();
			
			if (tempEstimatedPose.isEmpty()) {
				continue;
			}

			VisionEstimate estimatedPose = tempEstimatedPose.get();

			// Update the estimatedPose's std devs
			estimatedPose.stdDevs = updateEstimateStdDevs(estimatedPose, camera);

			RobotContainer.swerveSubsystem.addVisionMeasurement(estimatedPose);

			visionEstimates[index] = estimatedPose;
        }
    }
}
