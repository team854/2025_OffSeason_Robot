package frc.robot.subsystems.telemetry;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        if (Constants.DebugConstants.DEBUG_ELEVATOR) {
            sendElevatorTelemetry();
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
}
