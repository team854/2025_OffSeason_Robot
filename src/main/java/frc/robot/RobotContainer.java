// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.elevator.ControlElevatorBothStagesCommand;
import frc.robot.subsystems.arm.EndEffectorSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.telemetry.RobotAnimationSubsystem;
import frc.robot.subsystems.telemetry.SmartDashboardSubsystem;
import frc.robot.utilities.controls.CustomCommandXboxController;
import swervelib.SwerveInputStream;

public class RobotContainer {

	// Initalize public subsystems
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public static final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
	public static final WristSubsystem wristSubsystem = new WristSubsystem();
	public static final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
	public static final RobotAnimationSubsystem robotAnimationSubsystem = new RobotAnimationSubsystem();
	public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

	// Initalize driver controller
	public static final CustomCommandXboxController driverController = new CustomCommandXboxController(
			Constants.DriverConstants.PORT, Constants.DriverConstants.DEADBAND,
			Constants.DriverConstants.LEFT_JOYSTICK_EXPONENT,
			Constants.DriverConstants.RIGHT_JOYSTICK_EXPONENT, Constants.DriverConstants.TRIGGER_EXPONENT);

	public static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive,
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX())
			.withControllerRotationAxis(() -> -driverController.getRightX() * Constants.DriverConstants.ROTATION_SCALE)
			.deadband(0)
			.scaleTranslation(Constants.DriverConstants.TRANSLATION_SCALE)
			.allianceRelativeControl(false);

	public RobotContainer() {
			System.out.println("Configuring robot container");

			configureBindings();
	}

	private void configureBindings() {

		/*
		* Shoulder controls
		*/
		driverController.povUpDirection().whileTrue(shoulderSubsystem.setShoulderSpeedCommand(Constants.DriverConstants.CONTROL_SHOULDER_SPEED));
		driverController.povDownDirection().whileTrue(shoulderSubsystem.setShoulderSpeedCommand(Constants.DriverConstants.CONTROL_SHOULDER_SPEED.unaryMinus()));

		/*
		* Wrist controls
		*/
		driverController.povRightDirection().whileTrue(wristSubsystem.setWristSpeedCommand(Constants.DriverConstants.CONTROL_WRIST_SPEED));
		driverController.povLeftDirection().whileTrue(wristSubsystem.setWristSpeedCommand(Constants.DriverConstants.CONTROL_WRIST_SPEED.unaryMinus()));

		/*
		* Default commands
		*/
		Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOrientedSupplier(driveAngularVelocity);
		swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		elevatorSubsystem.setDefaultCommand(new ControlElevatorBothStagesCommand(() -> -driverController.getRightY()));



		System.out.println("Configured bindings");
	}

	public static boolean isBlueAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() != DriverStation.Alliance.Red : false;
    }

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
