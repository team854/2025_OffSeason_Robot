package frc.robot.commands.auto;
import java.util.function.Consumer;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.controls.RumbleUtilities;

public class AutoCoastCommand extends Command {

    public AutoCoastCommand() {
        addRequirements(
                RobotContainer.elevatorSubsystem,
                RobotContainer.shoulderSubsystem,
                RobotContainer.wristSubsystem);
    }

    private void setBrakeState(boolean idle) {
        IdleMode idleMode = idle ? IdleMode.kCoast : IdleMode.kBrake;

        RobotContainer.shoulderSubsystem.setCalibrationEnabled(idle);
        RobotContainer.elevatorSubsystem.setCalibrationEnabled(idle);
        RobotContainer.wristSubsystem.setCalibrationEnabled(idle);

        SparkMaxConfig shoulderMotorConfig = RobotContainer.shoulderSubsystem.getCurrentShoulderConfig();
        shoulderMotorConfig.idleMode(idleMode);
        RobotContainer.shoulderSubsystem.setShoulderMotorConfig(shoulderMotorConfig);

        SparkMaxConfig wristMotorConfig = RobotContainer.wristSubsystem.getCurrentWristConfig();
        wristMotorConfig.idleMode(idleMode);
        RobotContainer.wristSubsystem.setWristMotorConfig(wristMotorConfig);

        SparkMaxConfig stage1MotorConfig = RobotContainer.elevatorSubsystem.getCurrentStage1Config();
        stage1MotorConfig.idleMode(idleMode);
        RobotContainer.elevatorSubsystem.setStage1MotorConfig(stage1MotorConfig);

        SparkMaxConfig stage2MotorConfig = RobotContainer.elevatorSubsystem.getCurrentStage2Config();
        stage2MotorConfig.idleMode(idleMode);
        RobotContainer.elevatorSubsystem.setStage2MotorConfig(stage2MotorConfig);


    }

    @Override
    public void initialize() {
        setBrakeState(true);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        setBrakeState(false);
    }
}
