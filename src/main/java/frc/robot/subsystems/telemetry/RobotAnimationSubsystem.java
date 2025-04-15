package frc.robot.subsystems.telemetry;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.math.PoseUtilities;

public class RobotAnimationSubsystem extends SubsystemBase {
    public RobotAnimationSubsystem() {

    }

    @Override
    public void periodic() {
        if (!Constants.DebugConstants.ANIMATE_ROBOT) {
            return;
        }
        
        double elevatorStage1Height = RobotContainer.elevatorSubsystem.getStage1Height().in(Meter);
        double elevatorStage2Height = RobotContainer.elevatorSubsystem.getStage2Height().in(Meter);

        SmartDashboard.putNumberArray("Elevator/Stage1/Position", PoseUtilities.convertPoseToNumbers(new Pose3d(0, 0, elevatorStage1Height, new Rotation3d())));
        SmartDashboard.putNumberArray("Elevator/Stage2/Position", PoseUtilities.convertPoseToNumbers(new Pose3d(0, 0, elevatorStage1Height + elevatorStage2Height, new Rotation3d())));
    }
}
