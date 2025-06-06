package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.files.FileUtilities;
import frc.robot.utilities.files.JsonUtilities;
import frc.robot.utilities.pathfinding.LocalADStarAK;

public class PathfindingSubsystem extends SubsystemBase {
    public RobotConfig pathPlannerConfig;

    public PathfindingSubsystem() {
        createPathPlanner();
        System.out.println("Created PathfindingSubsystem");
    }

    private void createPathPlanner() {
        System.out.println("Syncing pathplanner config with constants");
        String pathPlannerFile = FileUtilities.readFile("pathplanner/settings.json");
        Map<String, Object> pathPlannerJson = JsonUtilities.fromJson(pathPlannerFile);

        // Get a list of all the swerve module config classes
        List<Class<?>> swerveModules = List.of(
                Constants.SwerveConstants.SwerveModuleConstants.BackLeft.class,
                Constants.SwerveConstants.SwerveModuleConstants.BackRight.class,
                Constants.SwerveConstants.SwerveModuleConstants.FrontLeft.class,
                Constants.SwerveConstants.SwerveModuleConstants.FrontRight.class);
    
        String pathPlannerModified = JsonUtilities.pathPlannerToJson(pathPlannerJson, swerveModules);

        System.out.println("Saving synced pathplanner config");
        FileUtilities.writeFile("pathplanner/settings.json", pathPlannerModified);

        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> RobotContainer.swerveSubsystem.getPose(), // Robot pose supplier
                // Method to reset odometry (will be called if your auto has a starting pose)
                initialHolonomicPose -> RobotContainer.swerveSubsystem.resetOdometry(initialHolonomicPose),
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                () -> RobotContainer.swerveSubsystem.getRobotVelocity(),
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (Constants.SwerveConstants.ENABLE_FEED_FORWARD) {
                        RobotContainer.swerveSubsystem.swerveDrive.drive(
                                speedsRobotRelative,
                                RobotContainer.swerveSubsystem.swerveDrive.kinematics
                                        .toSwerveModuleStates(speedsRobotRelative),
                                moduleFeedForwards.linearForces());
                    } else {
                        RobotContainer.swerveSubsystem.setChassisSpeeds(speedsRobotRelative);
                    }
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                // optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                        // holonomic drive trains
                        new PIDConstants(Constants.SwerveConstants.TranslationPID.P,
                                Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D), // Translation PID constants
                        new PIDConstants(Constants.SwerveConstants.HeadingPID.P, Constants.SwerveConstants.HeadingPID.I,
                                Constants.SwerveConstants.HeadingPID.D) // Rotation PID constants
                ),
                pathPlannerConfig, // The robot configuration
                () -> !RobotContainer.isBlueAlliance(),
                RobotContainer.swerveSubsystem // Reference to this subsystem to set requirements
        );

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            RobotContainer.smartDashboardSubsystem.pathplannerPathTelemetryCallback(activePath);
        });

        PathfindingCommand.warmupCommand().schedule();
    }

    public PathConstraints getPathConstraints() {
        return new PathConstraints(Constants.AutoConstants.TRANSLATION_MAX_VELOCITY.in(MetersPerSecond),
                Constants.AutoConstants.TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond),
                Constants.AutoConstants.ROTATION_MAX_VELOCITY.in(RadiansPerSecond),
                Constants.AutoConstants.ROTATION_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    }
}
