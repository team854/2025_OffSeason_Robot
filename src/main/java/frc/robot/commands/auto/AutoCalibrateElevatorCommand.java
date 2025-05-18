package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degree;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.controls.RumbleUtilities;

public class AutoCalibrateElevatorCommand extends Command {
    private double elevatorZeroOffset;
    private SequentialCommandGroup commands;

    public AutoCalibrateElevatorCommand() {
        addRequirements(
                RobotContainer.elevatorSubsystem,
                RobotContainer.shoulderSubsystem,
                RobotContainer.wristSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Calibrating elevator");

        this.commands = new SequentialCommandGroup();

        setupCommands();

        this.commands.initialize();
    }

    private void setupCommands() {
        Consumer<Double> elevatorZeroOffsetConsumer = (value) -> {
            this.elevatorZeroOffset = value;
        };

        Command elevatorCalibrationCommand = RobotContainer.elevatorSubsystem
                .calibrateElevatorCommand(elevatorZeroOffsetConsumer);

        this.commands.addCommands(elevatorCalibrationCommand);
    }

    @Override
    public void execute() {
        // If a direction is held then cancel this command
        if (!RobotContainer.driverController.noDirectionsHeld()) {
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

        if (!interrupted) {
            RumbleUtilities.rumbleCommandFullControlGiven();
            System.out.println("Elevator zero offset: " + this.elevatorZeroOffset + " meters");
            
            System.out.println("Elevator calibration command finished");
        }
    }
}
