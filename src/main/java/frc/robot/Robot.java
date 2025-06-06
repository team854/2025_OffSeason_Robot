// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot  {
	private Command autonomousCommand;

	private final RobotContainer robotContainer;

	public Robot() {
		// Set up advantage kit
		Logger.recordMetadata("ProjectName", "FirebladeRobotBase");
		Logger.addDataReceiver(new NT4Publisher());

		Logger.start();

		// Adjust loop overrun warning timeout
		try {
			Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
			watchdogField.setAccessible(true);
			Watchdog watchdog = (Watchdog) watchdogField.get(this);
			watchdog.setTimeout(0.2);
		} catch (Exception e) {
			DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
		}
		CommandScheduler.getInstance().setPeriod(0.2);

		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {
		RobotContainer.elevatorSubsystem.resetStage1Setpoint();
		RobotContainer.elevatorSubsystem.resetStage2Setpoint();

		// Set the shoulder to its current angle
		RobotContainer.shoulderSubsystem.resetShoulderSetpoint();

		RobotContainer.wristSubsystem.resetWristSetpoint();
	}

	@Override
	public void autonomousInit() {
		// Set the elevator stages to their current height
		RobotContainer.elevatorSubsystem.resetStage1Setpoint();
		RobotContainer.elevatorSubsystem.resetStage2Setpoint();

		// Set the shoulder to its current angle
		RobotContainer.shoulderSubsystem.resetShoulderSetpoint();

		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		} else {
			System.out.println("Canceled auto: No auto selected");
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		System.out.println("Finished autonomous");
	}

	@Override
	public void teleopInit() {
		// Set the elevator stages to their current height
		RobotContainer.elevatorSubsystem.resetStage1Setpoint();
		RobotContainer.elevatorSubsystem.resetStage2Setpoint();

		// Set the shoulder to its current angle
		RobotContainer.shoulderSubsystem.resetShoulderSetpoint();

		RobotContainer.wristSubsystem.resetWristSetpoint();

		RobotContainer.driveControlMode = RobotContainer.modeChooser.getSelected();
		RobotContainer.shoulderSubsystem.setMaxControlAngularVelocity(RobotContainer.isNormalMode() ? DegreesPerSecond.of(10000) : Constants.DriverConstants.BABY_CONTROL_SHOULDER_LIMIT_SPEED);
		RobotContainer.wristSubsystem.setMaxControlAngularVelocity(RobotContainer.isNormalMode() ? DegreesPerSecond.of(10000) : Constants.DriverConstants.BABY_CONTROL_WRIST_LIMIT_SPEED);

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
