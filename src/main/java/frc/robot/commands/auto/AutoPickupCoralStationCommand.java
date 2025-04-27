package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToPoseCommand;
import frc.robot.commands.drive.PathfindToPoseCommand;
import frc.robot.objects.InverseKinematicState;
import frc.robot.objects.InverseKinematicStatus;
import frc.robot.utilities.controls.RumbleUtilities;
import frc.robot.utilities.field.CoralStationUtilities;
import frc.robot.utilities.field.FieldUtilities;
import frc.robot.utilities.field.TagUtilities;

public class AutoPickupCoralStationCommand extends Command{
	private final boolean canCancel;
	private SequentialCommandGroup commands;

    public AutoPickupCoralStationCommand(boolean canCancel) {
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
        int closestTagID = CoralStationUtilities.getClosestCoralStation(RobotContainer.swerveSubsystem.getPose());

		if (closestTagID == -1) {
            System.out.println("No close coral station tag");
			RumbleUtilities.rumbleCommandFailed();
            this.cancel();
            return;
        }

        // Alert the driver and log the pickup
        System.out.println("Picking up coral from coral station");
        RumbleUtilities.rumbleCommandFullControlTaken();

        Pose3d tagPose = TagUtilities.getTagPose(closestTagID, true);
        
        Pose2d pickupOffset = CoralStationUtilities.getPickupPose(tagPose.toPose2d(), FieldUtilities.getFieldSide(tagPose.toPose2d()));
        Pose3d endEffectorPose = new Pose3d(
                pickupOffset.getX(),
                pickupOffset.getY(),
                Constants.CoralStationConstants.VERTICAL_OFFSET.in(Meter),
                new Rotation3d(
                        0,
                        Constants.CoralStationConstants.PICK_UP_ANGLE.in(Radian),
                        pickupOffset.getRotation().getRadians()));

        // Calculate the minium distance bettween the robot center and the coral station
        Distance minimumRobotDistance = Constants.RobotKinematicConstants.LENGTH.div(2);

        // Using inverse kinematics compute the robots pose
        InverseKinematicState robotTargetState = RobotContainer.endEffectorSubsystem
                .calculateInverseKinematicState(endEffectorPose, minimumRobotDistance);

        if (robotTargetState.status() == InverseKinematicStatus.INVALID) {
            System.err.println("Nan value target detected in auto pickup from coral station");
            RumbleUtilities.rumbleCommandFailed();
            this.cancel();
            return;
        }

        // The pose to go to that is a bit away so the arm and elevator have time to move into position
        Pose2d intermediatePose = robotTargetState.chassisPose()
                .plus(new Transform2d(-0.3, 0, Rotation2d.fromDegrees(0)));

        this.commands.addCommands(
            new ParallelCommandGroup(
                new PathfindToPoseCommand(intermediatePose, 0),
                RobotContainer.shoulderSubsystem.gotoShoulderAngleCommand(robotTargetState.shoulderAngle()),
                RobotContainer.wristSubsystem.gotoWristAngleCommand(Degree.of(0)),
                RobotContainer.elevatorSubsystem.gotoOverallHeightCommand(robotTargetState.elevatorHeight())),
            new MoveToPoseCommand(robotTargetState.chassisPose(), false).withDeadline(RobotContainer.endEffectorSubsystem.intakeUntil(Constants.DriverConstants.INTAKE_SPEED, true, 6))
        );
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
		System.out.println("Pickup command finished");
        if (!interrupted) {
            RumbleUtilities.rumbleCommandFullControlGiven();
        }
	}
}
