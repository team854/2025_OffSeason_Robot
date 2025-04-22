package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final ProfiledPIDController translationXController;
    private final ProfiledPIDController translationYController;
    private final PIDController headingController;
    private final Pose2d goalPose;
    private final boolean endAtPose;
    private double translationX;
    private double translationY;
	private double heading;

    /**
     * 
     * @param goalPose The pose to move to
     * @param endAtPose If the command should end once the robot has reached the goal pose
     */
    public MoveToPoseCommand(Pose2d goalPose, boolean endAtPose) {
        this.goalPose = goalPose;
        this.endAtPose = endAtPose;
        
        TrapezoidProfile.Constraints translatioConstraints = new TrapezoidProfile.Constraints(maxVelocity, Constants.AutoConstants.TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));
        
        // Configure the translation pids
        this.translationXController = new ProfiledPIDController(
                Constants.SwerveConstants.TranslationPID.P,
                Constants.SwerveConstants.TranslationPID.I,
                Constants.SwerveConstants.TranslationPID.D,
                translatioConstraints);
        this.translationYController = new ProfiledPIDController(
                Constants.SwerveConstants.TranslationPID.P,
                Constants.SwerveConstants.TranslationPID.I,
                Constants.SwerveConstants.TranslationPID.D,
                translatioConstraints);

        // Configure the heading pid and enable continuous input
        this.headingController = new PIDController(
                Constants.SwerveConstants.HeadingPID.P,
                Constants.SwerveConstants.HeadingPID.I,
                Constants.SwerveConstants.HeadingPID.D);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(RobotContainer.swerveSubsystem);
    }

    public void setGoalPose(Pose2d goalPose) {
        MoveToPoseCommand.globalGoalPose = goalPose;

        this.translationXController.setGoal(goalPose.getX());
        this.translationYController.setGoal(goalPose.getY());
	this.headingController.setSetpoint(goalPose.getRotation().getRadians());
    }

    @Override
    public void initialize() {
        this.setGoalPose(this.goalPose);

        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        // Reset the pids
        this.translationXController.reset(robotPose.getX());
        this.translationYController.reset(robotPose.getY());
        this.headingController.reset();

        // Set the tolerance of the pids
        this.translationXController
                .setTolerance(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR.in(Meter));
        this.translationYController
                .setTolerance(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR.in(Meter));
        this.headingController
                .setTolerance(Constants.SwerveConstants.ROTATION_ACCEPTABLE_ERROR.in(Radian));
        
        // Set the starting translation and heading values
        this.translationX = 0;
        this.translationY = 0;
		this.heading = 0;

        System.out.println("Moving to pose: " + MoveToPoseCommand.globalGoalPose.toString());
    }

    @Override
    public void execute() {
		Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

		// Save the old translations and heading 
		double oldTranslationX = this.translationX;
		double oldTranslationY = this.translationY;
		double oldHeading = this.heading;

		// Calculate the new translation and heading values
        this.translationX = MathUtil.clamp(this.translationXController.calculate(robotPose.getX()), -maxVelocity, maxVelocity);
        this.translationY = MathUtil.clamp(this.translationYController.calculate(robotPose.getY()), -maxVelocity, maxVelocity);
		this.heading = MathUtil.clamp(this.headingController
                .calculate(RobotContainer.swerveSubsystem.swerveDrive.getOdometryHeading().getRadians()), -maxAngularVelocity, maxAngularVelocity);

		// Calculate the delta in bettween the old and new translation and then divide it by a constant
		double finalTranslationX = this.translationX + ((this.translationX - oldTranslationX) / Constants.AutoConstants.TRANSLATION_FEEDFOWARD_DIVISOR);
		double finalTranslationY = this.translationY + ((this.translationY - oldTranslationY) / Constants.AutoConstants.TRANSLATION_FEEDFOWARD_DIVISOR);
		double finalHeading = this.heading + ((this.heading - oldHeading) / Constants.AutoConstants.ROTATION_FEEDFOWARD_DIVISOR);
        

        RobotContainer.swerveSubsystem.swerveDrive
                .driveFieldOriented(new ChassisSpeeds(finalTranslationX, finalTranslationY, finalHeading / 1.5));
    }

    @Override
    public boolean isFinished() {
        if (!endAtPose) {
            return false;
        }

        return this.translationXController.atGoal()
                && this.translationYController.atGoal()
                && this.headingController.atSetpoint()
                && RobotContainer.swerveSubsystem.isStopped();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.swerveSubsystem.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}
