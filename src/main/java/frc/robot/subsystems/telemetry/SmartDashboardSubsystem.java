package frc.robot.subsystems.telemetry;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToPoseCommand;
import frc.robot.utilities.math.PoseUtilities;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        if (Constants.DebugConstants.DEBUG_ELEVATOR) {
            sendElevatorTelemetry();
        }

        if (Constants.DebugConstants.DEBUG_SHOULDER) {
            sendShoulderTelemetry();
        }

        if (Constants.DebugConstants.DEBUG_WRIST) {
            sendWristTelemetry();
        }

        if (Constants.DebugConstants.DEBUG_PATHFINDING) {
            sendPathfindingTelemetry();
        }

        if (Constants.DebugConstants.DEBUG_END_EFFECTOR) {
            sendEndEffectorTelemetry();
        }
    }

    public void sendElevatorTelemetry() {
        SmartDashboard.putNumber("Elevator/Stage1/Setpoint", RobotContainer.elevatorSubsystem.getStage1Setpoint().in(Meter));
        SmartDashboard.putNumber("Elevator/Stage2/Setpoint", RobotContainer.elevatorSubsystem.getStage2Setpoint().in(Meter));

        SmartDashboard.putNumber("Elevator/Stage1/Height", RobotContainer.elevatorSubsystem.getStage1Height().in(Meter));
        SmartDashboard.putNumber("Elevator/Stage2/Height", RobotContainer.elevatorSubsystem.getStage2Height().in(Meter));

        SmartDashboard.putNumber("Elevator/Stage1/Velocity", RobotContainer.elevatorSubsystem.getStage1HeightVelocity().in(MetersPerSecond));
        SmartDashboard.putNumber("Elevator/Stage2/Velocity", RobotContainer.elevatorSubsystem.getStage2HeightVelocity().in(MetersPerSecond));
    }

    public void sendShoulderTelemetry() {
        SmartDashboard.putNumber("Arm/Shoulder/Angle", RobotContainer.shoulderSubsystem.getShoulderAngle().in(Degree));
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint", RobotContainer.shoulderSubsystem.getShoulderSetpoint().in(Degree));
        SmartDashboard.putNumber("Arm/Shoulder/Angular Velocity", RobotContainer.shoulderSubsystem.getShoulderVelocity().in(DegreesPerSecond));
    }

    public void sendWristTelemetry() {
        SmartDashboard.putNumber("Arm/Wrist/Angle", RobotContainer.wristSubsystem.getWristAngle().in(Degree));
        SmartDashboard.putNumber("Arm/Wrist/Setpoint", RobotContainer.wristSubsystem.getWristSetpoint().in(Degree));
        SmartDashboard.putNumber("Arm/Wrist/Angular Velocity", RobotContainer.wristSubsystem.getWristVelocity().in(DegreesPerSecond));
    }

    public void sendPathfindingTelemetry() {
        SmartDashboard.putNumberArray("Pathfinding/Move To Pose/Goal Pose", PoseUtilities.convertPoseToNumbers(MoveToPoseCommand.goalPose));
    }

    public void sendEndEffectorTelemetry() {
        
    }
}
