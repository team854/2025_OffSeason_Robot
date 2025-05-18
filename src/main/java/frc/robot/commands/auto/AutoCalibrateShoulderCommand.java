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

public class AutoCalibrateShoulderCommand extends Command {
    private double shoulderZeroOffset;
    private SequentialCommandGroup commands;

    public AutoCalibrateShoulderCommand() {
        addRequirements(
                RobotContainer.elevatorSubsystem,
                RobotContainer.shoulderSubsystem,
                RobotContainer.wristSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Calibrating shoulder");

        this.commands = new SequentialCommandGroup();

        setupCommands();

        this.commands.initialize();
    }

    private void setupCommands() {

        this.commands.addCommands(new ParallelCommandGroup(
                RobotContainer.elevatorSubsystem.gotoOverallHeightCommand(Constants.ArmConstants.LENGTH.plus(RobotContainer.elevatorSubsystem.getPivotPointOffset(true))),
                RobotContainer.wristSubsystem.gotoWristAngleCommand(Degree.of(0))));

        Consumer<Double> shoulderZeroOffsetConsumer = (value) -> {
            this.shoulderZeroOffset = value;
        };

        Command shoulderCalibrationCommand = RobotContainer.shoulderSubsystem
                .calibrateShoulderCommand(shoulderZeroOffsetConsumer);

        this.commands.addCommands(shoulderCalibrationCommand);
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
            System.out.println("Shoulder zero offset: " + this.shoulderZeroOffset);


            System.out.println("Shoulder calibration command finished");
        }
    }
}
