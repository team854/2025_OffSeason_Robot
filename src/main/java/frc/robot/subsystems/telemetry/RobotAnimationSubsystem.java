package frc.robot.subsystems.telemetry;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.math.PoseUtilities;

public class RobotAnimationSubsystem extends SubsystemBase {
	/*
	 * Constants
	 */
	private final double pivotPointOffset = RobotContainer.elevatorSubsystem.getPivotPointOffset(false).in(Meter);
	private final boolean isSimulation = Robot.isSimulation();

    public RobotAnimationSubsystem() {

    }

	@Override
    public void simulationPeriodic() {
		if (!Constants.TelemetryConstants.ANIMATIONS) {
            return;
        }

        Logger.recordOutput("FieldSimulation/Algae", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

	}

    @Override
    public void periodic() {
        if (!Constants.TelemetryConstants.ANIMATIONS) {
            return;
        }

		Pose3d robotPose = new Pose3d(RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get());
        
		/*
		 * Elevator
		 */

		// Get the heights of the elevator stages and its overall height
        double elevatorStage1Height = RobotContainer.elevatorSubsystem.getStage1Height().in(Meter);
        double elevatorStage2Height = RobotContainer.elevatorSubsystem.getStage2Height().in(Meter);
		double elevatorGroundHeight = elevatorStage1Height + elevatorStage2Height + pivotPointOffset;
		
		// Add the poses of the elevator stages to SmartDashboard
        SmartDashboard.putNumberArray("Elevator/Stage1/Position", PoseUtilities.convertPoseToNumbers(new Pose3d(0, 0, elevatorStage1Height, new Rotation3d())));
        SmartDashboard.putNumberArray("Elevator/Stage2/Position", PoseUtilities.convertPoseToNumbers(new Pose3d(0, 0, elevatorStage1Height + elevatorStage2Height, new Rotation3d())));

		/*
		 * Shoulder
		 */

		// Get the angle of the shoulder
		double shoulderAngle = RobotContainer.shoulderSubsystem.getShoulderAngle().in(Radian);

		// Calculate the rotation and the pose of the shoulder
		Rotation3d shoulderRotation = new Rotation3d(
				0,
				-shoulderAngle,
				0);
		
        Pose3d shoulderPose = new Pose3d(
				Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD.in(Meter),
        		0,
				elevatorGroundHeight, shoulderRotation);

		// Add the pose of the shoulder to SmartDashboard
		SmartDashboard.putNumberArray("Arm/Shoulder/Position", PoseUtilities.convertPoseToNumbers(shoulderPose));

		/*
		 * Wrist
		 */

		// Get the angle of the wrist
		double wristAngle = RobotContainer.wristSubsystem.getWristAngle().in(Radian);

		// Calculate the roation and the pose of the wrist
		Rotation3d wristRotation = new Rotation3d(
				wristAngle,
				-shoulderAngle,
				0);
		
        Pose3d wristPose = new Pose3d(
				Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD.in(Meter),
        		0.0023,
				elevatorGroundHeight, wristRotation);
				
		// Add the pose of the wrist to SmartDashboard
		SmartDashboard.putNumberArray("Arm/Wrist/Position", PoseUtilities.convertPoseToNumbers(wristPose));

		/*
		 * Coral
		 */

		if (RobotContainer.endEffectorSubsystem.hasCoral()) {
			Pose3d coralPose = RobotContainer.endEffectorSubsystem.calculateCoralPose(robotPose);

			if (isSimulation) {
				coralPose = coralPose.transformBy(new Transform3d(RobotContainer.endEffectorSubsystem.getClawSimulation().getCoralOffset().in(Meter), 0, 0, new Rotation3d()));
			}

			SmartDashboard.putNumberArray("Arm/Intake/Coral/Position", PoseUtilities.convertPoseToNumbers(coralPose));
		} else {
			SmartDashboard.putNumberArray("Arm/Intake/Coral/Position", PoseUtilities.convertPoseToNumbers(new Pose3d(5, 5, -5, new Rotation3d())));
		}
    }
}
