package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToPoseCommand;
import frc.robot.objects.InverseKinematicState;
import frc.robot.objects.InverseKinematicStatus;
import frc.robot.utilities.commands.ParallelRunAllDoneCommandGroup;
import frc.robot.utilities.controls.RumbleUtilities;
import frc.robot.utilities.field.ReefUtilities;
import frc.robot.utilities.field.TagUtilities;
import frc.robot.utilities.math.AngleUtilities;

public class AutoScoreCoralCommand extends Command {
	private final boolean right;
	private final int branchIndex;
	private final boolean canCancel;
	private SequentialCommandGroup commands;

	public AutoScoreCoralCommand(boolean right, int branchIndex, boolean canCancel) {
		this.right = right;
		this.branchIndex = branchIndex;
		this.canCancel = canCancel;

		// Set the entire robot as a requirement
		addRequirements(RobotContainer.shoulderSubsystem, RobotContainer.wristSubsystem,
				RobotContainer.elevatorSubsystem, RobotContainer.swerveSubsystem);
	}

	@Override
	public void initialize() {
		this.commands = new SequentialCommandGroup();

		setupCommands();

		this.commands.initialize();
	}

	private void setupCommands() {
		int closestTagID = ReefUtilities.getClosestReef(RobotContainer.swerveSubsystem.getPose());

		if (closestTagID == -1) {
			System.out.println("No close reef tag");
			RumbleUtilities.rumbleCommandFailed();
			this.cancel();
            return;
		}
		// Alert the driver and log the scoring
		System.out.println("Scoring on level " + (branchIndex + 1));
		RumbleUtilities.rumbleCommandFullControlTaken();

		Pose3d tagPose = TagUtilities.getTagPose(closestTagID, true);

		// Calculate the pose of the branch
		Pose2d branchOffset = ReefUtilities.getBranchPose(tagPose.toPose2d(), this.right);
		Pose3d branchPose = new Pose3d(
				branchOffset.getX(),
				branchOffset.getY(),
				ReefUtilities.getBranchHeight(branchIndex).in(Meter),
				new Rotation3d(
						0,
						ReefUtilities.getBranchAngle(branchIndex).in(Radian),
						branchOffset.getRotation().getRadians()));

		// Calculate the scoring angle (perpendicular to the branch angle unless its L1) 
		// Then calculate the new end effector pose
		Angle scoringAngle = branchPose.getRotation().getMeasureY();
		if (this.branchIndex != 0) {
			scoringAngle = AngleUtilities.getPerpendicularAngle(branchPose.getRotation().getMeasureY()).unaryMinus();
		}
		Pose3d endEffectorPose = new Pose3d(branchPose.getTranslation(),
				new Rotation3d(0, scoringAngle.in(Radian), branchPose.getRotation().getZ()));

		// Calculate the minium distance bettween the robot center and the branch
		Distance minimumRobotDistance = (Constants.RobotKinematicConstants.LENGTH.div(2))
				.plus(Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET);

		// Using inverse kinematics compute the robots pose
		InverseKinematicState robotTargetState = RobotContainer.endEffectorSubsystem
				.calculateInverseKinematicState(endEffectorPose, minimumRobotDistance);

		if (robotTargetState.status() == InverseKinematicStatus.INVALID) {
			System.err.println("Nan value target detected in auto score");
			RumbleUtilities.rumbleCommandFailed();
			this.cancel();
            return;
		}

		// The pose to go to that is a bit away so the arm and elevator have time to move into position
		Pose2d intermediatePose = robotTargetState.chassisPose()
				.plus(new Transform2d(-0.3, 0, Rotation2d.fromDegrees(0)));

		// The pose to go to when the robot is pulling away from the branch 
		Pose2d pullBackPose = robotTargetState.chassisPose()
				.plus(new Transform2d(-0.2, 0, Rotation2d.fromDegrees(0)));

		// Get the angle the wrist needs to be at to score
		Angle wristAngle = ReefUtilities.getWristAngle(this.branchIndex);

		// Get the shoulder approch angle
		Angle approchAngle = robotTargetState.shoulderAngle();
		if (this.branchIndex != 0) {
			// Add the lift angle when its not L1 so it clears the branchs before it puts the coral on it
			approchAngle = approchAngle.plus(Constants.ReefConstants.LIFT_ANGLE);
		}

		this.commands.addCommands(
				new ParallelRunAllDoneCommandGroup(
					new MoveToPoseCommand(intermediatePose, true),
					RobotContainer.elevatorSubsystem
								.gotoOverallHeightCommand(
										robotTargetState.elevatorHeight()),
								RobotContainer.shoulderSubsystem.gotoShoulderAngleCommand(approchAngle),
								RobotContainer.wristSubsystem.gotoWristAngleCommand(wristAngle)),
				new MoveToPoseCommand(robotTargetState.chassisPose(), true),
				RobotContainer.shoulderSubsystem.gotoShoulderAngleCommand(robotTargetState.shoulderAngle()),
				new ParallelCommandGroup(
					// There is no need to pull back if its L1
					new MoveToPoseCommand((this.branchIndex != 0) ? pullBackPose : robotTargetState.chassisPose(), true),
					RobotContainer.endEffectorSubsystem.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 5)
				));
	}

	@Override
	public void execute() {
		// If a direction is held and the comamnd can be cancelled then cancel this command
		if (!RobotContainer.driverController.noDirectionsHeld() && canCancel) {
			this.cancel();
		}
		this.commands.execute();
	}

	@Override
	public boolean isFinished() {
		return this.commands.isFinished();
	}

	@Override
	public void end(boolean interrupted) {
		this.commands.end(interrupted);
		System.out.println("Scoring command finished");
		if (!interrupted) {
			RumbleUtilities.rumbleCommandFullControlGiven();
		}
	}
}
