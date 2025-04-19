package frc.robot.commands.setpoints;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class GroundPickupConfigurationCommand extends ParallelCommandGroup {
    public GroundPickupConfigurationCommand() {
        addCommands(
            RobotContainer.shoulderSubsystem.setShoulderAngleCommand(Constants.SetpointConstants.GroundIntake.SHOULDER_ANGLE),
            RobotContainer.wristSubsystem.setWristAngleCommand(Constants.SetpointConstants.GroundIntake.WRIST_ANGLE),
            RobotContainer.elevatorSubsystem.setOverallHeightCommand(Constants.SetpointConstants.GroundIntake.ELEVATOR_GROUND_HEIGHT)
        );
    }
}
