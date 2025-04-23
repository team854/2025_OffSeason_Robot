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
     * Constants
     */
    private final double maxVelocity = Constants.AutoConstants.TRANSLATION_MAX_VELOCITY.in(MetersPerSecond);
    private final double maxAngularVelocity = Constants.AutoConstants.ROTATION_MAX_VELOCITY.in(RadiansPerSecond);

    /*
     * Public variable
     */
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
        
        TrapezoidProfile.Constraints translatioConstraints = new TrapezoidProfile.Constraints(maxVelocity, Constants.AutoConstants.TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));
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

        addRequirements(RobotContainer.swerveSubsystem);
    }

    public void setGoalPose(Pose2d goalPose) {
        MoveToPoseCommand.globalGoalPose = goalPose;
        this.goalPose = goalPose;

        this.translationController.setGoal(0);

        this.headingController.setGoal(goalPose.getRotation().getRadians());
    }
    
    private Distance getTargetDistance(Pose2d robotPose) {
        return Meter.of(robotPose.getTranslation().getDistance(this.goalPose.getTranslation()));
    }

    private Distance getTargetDistance(Translation2d robotPose) {
        return Meter.of(robotPose.getDistance(this.goalPose.getTranslation()));
    }

    @Override
    public void initialize() {
        this.setGoalPose(this.goalPose);

        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        // Reset the pids
        this.translationController.reset(getTargetDistance(robotPose).in(Meter));
        this.headingController.reset(robotPose.getRotation().getRadians());

        // Set the tolerance of the pids
        this.translationController
                .setTolerance(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR.in(Meter));
        this.headingController
                .setTolerance(Constants.SwerveConstants.ROTATION_ACCEPTABLE_ERROR.in(Radian));

        ChassisSpeeds curentSpeeds = RobotContainer.swerveSubsystem.swerveDrive.getFieldVelocity();

        this.oldTranslationSpeed = new Translation2d(curentSpeeds.vxMetersPerSecond, curentSpeeds.vyMetersPerSecond).getNorm();
        this.oldHeadingSpeed = curentSpeeds.omegaRadiansPerSecond;

        System.out.println("Moving to pose: " + MoveToPoseCommand.globalGoalPose.toString());
    }

    @Override
    public void execute() {
		Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        Distance targetDistance = getTargetDistance(robotPose);

        double rotationDifference = Math.atan2(this.goalPose.getY() - robotPose.getY(), this.goalPose.getX() - robotPose.getX());

        double targetTranslation = -this.translationController.calculate(targetDistance.in(Meter), 0);
        double heading = this.headingController
                .calculate(RobotContainer.swerveSubsystem.swerveDrive.getOdometryHeading().getRadians());
        
        double translationDelta = (targetTranslation - this.oldTranslationSpeed) / Constants.AutoConstants.TRANSLATION_FEEDFOWARD_DIVISOR;
        double headingDelta = (heading - this.oldHeadingSpeed) / Constants.AutoConstants.ROTATION_FEEDFOWARD_DIVISOR;

        double translationX = (targetTranslation + translationDelta) * Math.cos(rotationDifference);
        double translationY = (targetTranslation + translationDelta) * Math.sin(rotationDifference);
        

        RobotContainer.swerveSubsystem.swerveDrive
                .driveFieldOriented(new ChassisSpeeds(translationX, translationY, heading + headingDelta));

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
        RobotContainer.swerveSubsystem.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}
