package frc.robot.subsystems.telemetry;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToPoseCommand;
import frc.robot.commands.drive.PathfindToPoseCommand;
import frc.robot.objects.VisionEstimate;
import frc.robot.subsystems.simulation.CoralStationSimulation;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.utilities.math.PoseUtilities;
import frc.robot.utilities.saftey.ArmSafteyUtilities;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        if (Constants.TelemetryConstants.ELEVATOR_TELEMETRY) {
            sendElevatorTelemetry();
        }

        if (Constants.TelemetryConstants.SHOULDER_TELEMETRY) {
            sendShoulderTelemetry();
        }

        if (Constants.TelemetryConstants.WRIST_TELEMETRY) {
            sendWristTelemetry();
        }

        if (Constants.TelemetryConstants.PATHFINDING_TELEMETRY) {
            sendPathfindingTelemetry();
        }

        if (Constants.TelemetryConstants.END_EFFECTOR_TELEMETRY) {
            sendEndEffectorTelemetry();
        }

        if (Constants.TelemetryConstants.VISION_TELEMETRY) {
            sendVisionTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (Constants.TelemetryConstants.SIMULATION_TELEMETRY) {
            sendSimulationTelemetry();
        }
    }

    public void sendElevatorTelemetry() {
        SmartDashboard.putNumber("Elevator/Stage1/Setpoint", RobotContainer.elevatorSubsystem.getStage1Setpoint().in(Meter));
        SmartDashboard.putNumber("Elevator/Stage2/Setpoint", RobotContainer.elevatorSubsystem.getStage2Setpoint().in(Meter));

        SmartDashboard.putNumber("Elevator/Stage1/Height", RobotContainer.elevatorSubsystem.getStage1Height().in(Meter));
        SmartDashboard.putNumber("Elevator/Stage2/Height", RobotContainer.elevatorSubsystem.getStage2Height().in(Meter));

        SmartDashboard.putNumber("Elevator/Stage1/Velocity", RobotContainer.elevatorSubsystem.getStage1HeightVelocity().in(MetersPerSecond));
        SmartDashboard.putNumber("Elevator/Stage2/Velocity", RobotContainer.elevatorSubsystem.getStage2HeightVelocity().in(MetersPerSecond));

        SmartDashboard.putNumberArray("Elevator/Lower Effector Limit", ArmSafteyUtilities.generateDebugLine());

        SmartDashboard.putNumber("Elevator/Stage1/Target Voltage", RobotContainer.elevatorSubsystem.getStage1MotorVoltage().in(Volt));
        SmartDashboard.putNumber("Elevator/Stage2/Target Voltage", RobotContainer.elevatorSubsystem.getStage2MotorVoltage().in(Volt));
    }

    public void sendShoulderTelemetry() {
        SmartDashboard.putNumber("Arm/Shoulder/Angle", RobotContainer.shoulderSubsystem.getShoulderAngle().in(Degree));
        SmartDashboard.putNumber("Arm/Shoulder/Setpoint", RobotContainer.shoulderSubsystem.getShoulderSetpoint().in(Degree));
        SmartDashboard.putNumber("Arm/Shoulder/Angular Velocity", RobotContainer.shoulderSubsystem.getShoulderVelocity().in(DegreesPerSecond));
        SmartDashboard.putNumber("Arm/Shoulder/Target Voltage", RobotContainer.shoulderSubsystem.getShoulderMotorVoltage().in(Volt));
    }

    public void sendWristTelemetry() {
        SmartDashboard.putNumber("Arm/Wrist/Angle", RobotContainer.wristSubsystem.getWristAngle().in(Degree));
        SmartDashboard.putNumber("Arm/Wrist/Setpoint", RobotContainer.wristSubsystem.getWristSetpoint().in(Degree));
        SmartDashboard.putNumber("Arm/Wrist/Angular Velocity", RobotContainer.wristSubsystem.getWristVelocity().in(DegreesPerSecond));
    }

    public void sendPathfindingTelemetry() {
        SmartDashboard.putNumberArray("Pathfinding/Move To Pose/Goal Pose", PoseUtilities.convertPoseToNumbers(MoveToPoseCommand.globalGoalPose));

        SmartDashboard.putNumberArray("Pathfinding/Pathfind To Pose/Goal Pose", PoseUtilities.convertPoseToNumbers(PathfindToPoseCommand.globalGoalPose));
    }

    public void pathplannerPathTelemetryCallback(List<Pose2d> activePath) {
        SmartDashboard.putNumberArray("Pathfinding/Pathfind To Pose/Active Path", PoseUtilities.convertPoseArrayToNumbers(activePath.toArray(new Pose2d[0])));
    }

    public void sendEndEffectorTelemetry() {
        SmartDashboard.putBoolean("Arm/Intake/Has Coral", RobotContainer.endEffectorSubsystem.hasCoral());
        SmartDashboard.putNumber("Arm/Intake/Motor Percent", RobotContainer.endEffectorSubsystem.getIntakeSpeedPercent());

        Pose3d endEffectorPose = RobotContainer.endEffectorSubsystem.calculateEndEffectorPose();
        SmartDashboard.putNumberArray("Arm/End Effector Pose", PoseUtilities.convertPoseToNumbers(endEffectorPose));
    }

    public void sendVisionTelemetry() {
        for (int index = 0; index < RobotContainer.visionSubsystem.visionCameras.length; index++) {
            // Retrive the camera and the estimate from the vision subsystem
			VisionCamera camera = RobotContainer.visionSubsystem.visionCameras[index];
            VisionEstimate estimate = RobotContainer.visionSubsystem.visionEstimates[index];

            // Construct the path prefix
            String prefixString = "PhotonVision/" + camera.getCameraName();

            SmartDashboard.putNumberArray(prefixString + "/Estimate Std Devs",
                estimate.stdDevs.getData());

            SmartDashboard.putNumberArray(prefixString + "/Estimate Robot Pose",
                PoseUtilities.convertPoseToNumbers(estimate.estimatedPose));

            SmartDashboard.putNumber(prefixString + "/Estimate Timestamp",
                estimate.timestampSeconds);

            SmartDashboard.putNumberArray(prefixString + "/Camera Pose",
                PoseUtilities.convertPoseToNumbers(camera.getCameraPose()));
        }
    }

    public void sendSimulationTelemetry() {
        SmartDashboard.putNumberArray("Simulation/End Effector/Pickup Volume", RobotContainer.endEffectorSubsystem.getClawSimulation().getPickupVolume().generateDebugWireframe());

        CoralStationSimulation[] coralStationArray = RobotContainer.simulationSubsystem.getCoralStations();
        for (int index = 0; index < coralStationArray.length; index++) {
            SmartDashboard.putNumberArray("Simulation/Coral Stations/Station " + index, coralStationArray[index].getSpawnVolume().generateDebugWireframe());
        }
    }
}
