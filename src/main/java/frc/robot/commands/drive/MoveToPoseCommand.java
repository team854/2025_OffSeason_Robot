package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MoveToPoseCommand extends Command {
    /*
     * Public variable
     */
    // The logging system pulls from this variable
    public static Pose2d globalGoalPose = new Pose2d();

    /*
     * Control
     */
    private final ProfiledPIDController translationController;
    private final ProfiledPIDController headingController;
    private final boolean endAtPose;
    private Pose2d goalPose;
    private double oldTranslationSpeed;
    private double oldHeadingSpeed;

    /**
     * 
     * @param goalPose The pose to move to
     * @param endAtPose If the command should end once the robot has reached the goal pose
     */
    public MoveToPoseCommand(Pose2d goalPose, boolean endAtPose) {
        this.goalPose = goalPose;
        this.endAtPose = endAtPose;
        
        TrapezoidProfile.Constraints translatioConstraints = new TrapezoidProfile.Constraints(Constants.AutoConstants.TRANSLATION_MAX_VELOCITY.in(MetersPerSecond), Constants.AutoConstants.TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));
        TrapezoidProfile.Constraints headingConstraints = new TrapezoidProfile.Constraints(Constants.AutoConstants.ROTATION_MAX_VELOCITY.in(RadiansPerSecond), Constants.AutoConstants.ROTATION_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
        
        // Configure the translation pid
        this.translationController = new ProfiledPIDController(
                Constants.SwerveConstants.TranslationPID.P,
                Constants.SwerveConstants.TranslationPID.I,
                Constants.SwerveConstants.TranslationPID.D,
                translatioConstraints);

        // Configure the heading pid and enable continuous input
        this.headingController = new ProfiledPIDController(
                Constants.SwerveConstants.HeadingPID.P,
                Constants.SwerveConstants.HeadingPID.I,
                Constants.SwerveConstants.HeadingPID.D,
                headingConstraints);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Prevent two systems from accessing the swerver subsystem at once
        addRequirements(RobotContainer.swerveSubsystem);
    }

    public void setGoalPose(Pose2d goalPose) {
        // Update both goal poses
        MoveToPoseCommand.globalGoalPose = goalPose;
        this.goalPose = goalPose;

        // Update the goals in the pids
        this.translationController.setGoal(0);
        this.headingController.setGoal(goalPose.getRotation().getRadians());
    }
    
    private Distance getTargetDistance(Pose2d robotPose) {
        return Meter.of(robotPose.getTranslation().getDistance(this.goalPose.getTranslation()));
    }

    @Override
    public void initialize() {
        this.setGoalPose(this.goalPose);

        // Get the current pose of the robot
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        // Reset the pids
        this.translationController.reset(getTargetDistance(robotPose).in(Meter));
        this.headingController.reset(robotPose.getRotation().getRadians());

        // Set the tolerance of the pids
        this.translationController
                .setTolerance(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR.in(Meter));
        this.headingController
                .setTolerance(Constants.SwerveConstants.ROTATION_ACCEPTABLE_ERROR.in(Radian));

        // Get the current speed of the robot relative to the field
        ChassisSpeeds curentSpeeds = RobotContainer.swerveSubsystem.swerveDrive.getFieldVelocity();

        // Prevent abrupt transitions if the robot is already moving
        this.oldTranslationSpeed = Math.hypot(curentSpeeds.vxMetersPerSecond, curentSpeeds.vyMetersPerSecond);
        this.oldHeadingSpeed = curentSpeeds.omegaRadiansPerSecond;

        System.out.println("Moving to pose: " + this.goalPose.toString());
    }

    @Override
    public void execute() {
        // Get the current pose of the robot
		Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        // Get the distance bettween the target and the robot
        Distance targetDistance = getTargetDistance(robotPose);

        // Calculate the translation differnce bettween the robot and the target and normalize it
        Translation2d translationDifference = robotPose.getTranslation().minus(this.goalPose.getTranslation()).div(targetDistance.in(Meter));

        // Get the translation controller's target speed and the heading controllers target rotational speed
        double targetTranslation = this.translationController.calculate(targetDistance.in(Meter), 0);
        double heading = this.headingController
                .calculate(RobotContainer.swerveSubsystem.swerveDrive.getOdometryHeading().getRadians());

        // Calculate the delta bettween the previous input and the current input
        // This acts sort of like a feed foward controller as if the speed goes up we know more power will be needed
        // and this helps the control system react a little bit quicker
        double translationDelta = (targetTranslation - this.oldTranslationSpeed) / Constants.AutoConstants.TRANSLATION_FEEDFOWARD_DIVISOR;
        double headingDelta = (heading - this.oldHeadingSpeed) / Constants.AutoConstants.ROTATION_FEEDFOWARD_DIVISOR;

        // Ge the final target speeds
        Translation2d finalTranslation = translationDifference.times(targetTranslation + translationDelta);

        RobotContainer.swerveSubsystem.swerveDrive
                .driveFieldOriented(new ChassisSpeeds(finalTranslation.getX(), finalTranslation.getY(), heading + headingDelta));

        // Update the previous translation and heading speeds to the current speeds
        this.oldTranslationSpeed = targetTranslation;
        this.oldHeadingSpeed = heading;
    }

    @Override
    public boolean isFinished() {
        if (!endAtPose) {
            return false;
        }

        return this.translationController.atGoal()
                && this.headingController.atSetpoint()
                && RobotContainer.swerveSubsystem.isStopped();
    }

    @Override
    public void end(boolean interrupted) {
        // Reset the logging so it shows nothing when its not moving to pose
        MoveToPoseCommand.globalGoalPose = new Pose2d();

        // Prevent the robot from continuing in the direction it was last commanded to go in
        RobotContainer.swerveSubsystem.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}
