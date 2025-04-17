package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToPoseCommand;
import frc.robot.objects.InverseKinematicState;

public class DemoEndEffector extends Command {
    private Translation3d rotatePose;
    private double pitchAngle = 70;
    private double yawAngle;
    private MoveToPoseCommand pathCommand;

    public DemoEndEffector() {

        addRequirements(RobotContainer.shoulderSubsystem, RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        pathCommand = new MoveToPoseCommand(RobotContainer.swerveSubsystem.getPose(), false);
        pathCommand.initialize();
        rotatePose = new Translation3d(RobotContainer.swerveSubsystem.getPose().getX(), RobotContainer.swerveSubsystem.getPose().getY(), 1);
        yawAngle = RobotContainer.swerveSubsystem.getPose().getRotation().getRadians();
        pitchAngle = 70;
    }

    @Override
    public void execute() {
        pitchAngle -= 10.0 / 50;

        Pose3d newPose = new Pose3d(rotatePose, new Rotation3d(0, Units.degreesToRadians(pitchAngle), yawAngle));

        InverseKinematicState fullRobotTargetState = RobotContainer.endEffectorSubsystem
					.calculateInverseKinematicState(newPose, Meter.of(0));

        pathCommand.setNewPose(fullRobotTargetState.chassisPose());

        pathCommand.execute(); 

        RobotContainer.shoulderSubsystem.setShoulderSetpoint(fullRobotTargetState.shoulderAngle());

        RobotContainer.elevatorSubsystem.setOverallHeight(fullRobotTargetState.elevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return pitchAngle <= -70;
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
        System.out.println("ENDING");
        this.cancel();
    }
}
