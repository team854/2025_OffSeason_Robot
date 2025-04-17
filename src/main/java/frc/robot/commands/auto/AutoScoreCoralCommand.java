package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoScoreCoralCommand extends Command {
    private final boolean right;
	private final int branchIndex;
	private final boolean canCancel;
	private SequentialCommandGroup commands;

    public AutoScoreCoralCommand(boolean right, int branchIndex, boolean canCancel) {
        this.right = right;
		this.branchIndex = branchIndex;
		this.canCancel = canCancel;

		addRequirements(RobotContainer.shoulderSubsystem, RobotContainer.wristSubsystem, RobotContainer.elevatorSubsystem, RobotContainer.swerveSubsystem);
    }

    @Override
	public void initialize() {
		this.commands = new SequentialCommandGroup();
        FINISH ME
    }
}
