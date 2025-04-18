package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.files.JsonUtilities;

public class VisionSubsystem extends SubsystemBase {
    public List<VisionCamera> limelightCameras = new ArrayList<>();

    public VisionSystemSim visionSim;

    private boolean isSimulation = Robot.isSimulation();

    public VisionSubsystem() {
        boolean anyCameras = setupCameras();

        if (!anyCameras) {
            System.out.println("No cameras configured");;
        }
    }

    private boolean setupCameras() {
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

                    limelightCameras.add(new VisionCamera(limelightName,
                            new Transform3d(limelightRelativePosition, limelightRelativeRotation), width, height, fps,
                            diagonal_fov, average_pixel_error, average_pixel_error_std_devs, average_latency,
                            average_latency_std_devs, effectiveRange,
                            visionSim));

                    System.out.println("Camera with name " + limelightName + " is enabled");
                } else {
                    System.out.println("Camera with name " + limelightName + " is not enabled");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return limelightCameras.size() > 0;
    }
}
