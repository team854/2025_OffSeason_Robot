package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.DriveControlMode;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class ControlElevatorBothStagesCommand extends Command {
    private final Supplier<Double> elevatorSpeed;
    private final double elevatorOffset = RobotContainer.elevatorSubsystem.getPivotPointOffset(true).in(Meter);
    private double lastOn = 0;

    public ControlElevatorBothStagesCommand(Supplier<Double> elevatorSpeed) {
        this.elevatorSpeed = elevatorSpeed;

        addRequirements(RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (RobotContainer.driveControlMode != DriveControlMode.NORMAL) {
            return;
        }

        if (Math.abs(elevatorSpeed.get()) < 0.04) {
            if (lastOn != 0) {
                lastOn = 0;
                
                double addVelocity = 5 * ((RobotContainer.elevatorSubsystem.getStage1HeightVelocity().in(MetersPerSecond) + RobotContainer.elevatorSubsystem.getStage2HeightVelocity().in(MetersPerSecond)) / 50);
                double currentHeight = (RobotContainer.elevatorSubsystem.getStage1Height().in(Meter) + RobotContainer.elevatorSubsystem.getStage2Height().in(Meter));
                RobotContainer.elevatorSubsystem.setOverallHeight(Meter.of(currentHeight + addVelocity + elevatorOffset));
            }
            return;
        }
        lastOn = elevatorSpeed.get();
        RobotContainer.elevatorSubsystem.setOverallHeight(Meter.of(RobotContainer.elevatorSubsystem.getStage1Setpoint().in(Meter) + RobotContainer.elevatorSubsystem.getStage2Setpoint().in(Meter) + ((elevatorSpeed.get() * Constants.DriverConstants.CONTROL_ELEVATOR_SPEED.in(MetersPerSecond)) / 50) + elevatorOffset));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
