package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.VisionEstimate;
import frc.robot.utilities.files.FileUtilities;
import frc.robot.utilities.files.JsonUtilities;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    /*
     * Swerve
     */
    public SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        makeYagslSwerveConfig();

        createAndConfigureSwerve();

        System.out.println("Created SwerveSubsystem");
    }

    private void createAndConfigureSwerve() {

        Pose2d startingPose = new Pose2d(
                new Translation2d(Constants.Starting.X.in(Meter), Constants.Starting.Y.in(Meter)), new Rotation2d());

        try {
            System.out.println("Initalizing swerveDrive");
            swerveDrive = new SwerveParser(
                    new File(Filesystem.getDeployDirectory(), Constants.SwerveConstants.SWERVECONFIGDIR))
                    .createSwerveDrive(Constants.RobotKinematicConstants.MAX_SPEED.in(MetersPerSecond), startingPose);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, Constants.SwerveConstants.Imu.ANGULAR_VELOCITY_COEFF);
        swerveDrive.setGyroOffset(new Rotation3d(0, 0, Constants.SwerveConstants.Imu.OFFSET.in(Radian)));
    }

    private void makeYagslSwerveConfig() {
        System.out.println("Constructing YAGSL config objects");

        // Construct all the modules json based on the classes in Constants
        String backLeft = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.BackLeft.class);
        String backRight = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.BackRight.class);
        String frontLeft = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.FrontLeft.class);
        String frontRight = JsonUtilities
                .moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.FrontRight.class);

        String physicalProperties = JsonUtilities.physicalPropertiesToJson();
        String pidfProperties = JsonUtilities.pidfPropertiesToJson();
        String controllerProperties = JsonUtilities.controllerPropertiesToJson();
        String swerveDrive = JsonUtilities.swerveDriveToJson();

        System.out.println("Saving constructed YAGSL config objects");

        // Save the constructed objects
        FileUtilities.writeFile("swerve/modules/backleft.json", backLeft);
        FileUtilities.writeFile("swerve/modules/backright.json", backRight);
        FileUtilities.writeFile("swerve/modules/frontleft.json", frontLeft);
        FileUtilities.writeFile("swerve/modules/frontright.json", frontRight);

        FileUtilities.writeFile("swerve/modules/physicalproperties.json", physicalProperties);
        FileUtilities.writeFile("swerve/modules/pidfproperties.json", pidfProperties);
        FileUtilities.writeFile("swerve/controllerproperties.json", controllerProperties);
        FileUtilities.writeFile("swerve/swervedrive.json", swerveDrive);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOrientedSupplier(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public boolean isStopped() {
        return Math.abs(RobotContainer.swerveSubsystem
                .getRobotVelocity().omegaRadiansPerSecond) < Constants.SwerveConstants.ROTATION_ZERO_THRESHOLD
                        .in(RadiansPerSecond)
                && RobotContainer.swerveSubsystem.convertChassisSpeed(RobotContainer.swerveSubsystem
                        .getRobotVelocity())
                        .in(MetersPerSecond) < Constants.SwerveConstants.TRANSLATION_ZERO_THRESHOLD.in(MetersPerSecond);
    }

    /**
     * 
     * @param visionEstimate The vision estimate to add to odometry
     */
    public void addVisionMeasurement(VisionEstimate visionEstimate) {
        swerveDrive.addVisionMeasurement(visionEstimate.estimatedPose.toPose2d(), visionEstimate.timestampSeconds,
                visionEstimate.stdDevs);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public LinearVelocity convertChassisSpeed(ChassisSpeeds chassisSpeeds) {
        return MetersPerSecond.of(Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    }

    public AHRS getGyro() {
        return (AHRS) swerveDrive.getGyro().getIMU();
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
        System.out.println("Zeroed gyro");
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        // Update the pose estimates from the vision subsystem right after updating the odometry
        RobotContainer.visionSubsystem.updateVisionEstimates();
    }

}
