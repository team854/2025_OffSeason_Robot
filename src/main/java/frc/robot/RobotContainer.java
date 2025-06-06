// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoCalibrateElevatorCommand;
import frc.robot.commands.auto.AutoCalibrateShoulderCommand;
import frc.robot.commands.auto.AutoCoastCommand;
import frc.robot.commands.auto.AutoPickupCoralStationCommand;
import frc.robot.commands.auto.AutoScoreCoralCommand;
import frc.robot.commands.elevator.ControlElevatorBothStagesCommand;
import frc.robot.commands.setpoints.GroundPickupConfigurationCommand;
import frc.robot.commands.testing.DemoEndEffector;
import frc.robot.objects.DriveControlMode;
import frc.robot.subsystems.arm.EndEffectorSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.PathfindingSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.telemetry.RobotAnimationSubsystem;
import frc.robot.subsystems.telemetry.SmartDashboardSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.controls.CustomCommandXboxController;
import frc.robot.utilities.controls.RumbleUtilities;
import swervelib.SwerveInputStream;

public class RobotContainer {

	// Robot control mode
	public static DriveControlMode driveControlMode = DriveControlMode.NORMAL;

	// Initalize public subsystems
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public static final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
	public static final WristSubsystem wristSubsystem = new WristSubsystem();
	public static final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
	public static final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
	public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
	public static final PathfindingSubsystem pathfindingSubsystem = new PathfindingSubsystem();
	public static final RobotAnimationSubsystem robotAnimationSubsystem = new RobotAnimationSubsystem();
	public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
	public static SimulationSubsystem simulationSubsystem;

	// Initalize driver controller
	public static final CustomCommandXboxController driverController = new CustomCommandXboxController(
			Constants.DriverConstants.PORT,
			Constants.DriverConstants.DEADBAND,
			Constants.DriverConstants.LEFT_JOYSTICK_EXPONENT,
			Constants.DriverConstants.RIGHT_JOYSTICK_EXPONENT,
			Constants.DriverConstants.TRIGGER_EXPONENT);

	public static final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.swerveDrive,
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX())
			.withControllerRotationAxis(() -> -driverController.getRightX() * Constants.DriverConstants.ROTATION_SCALE)
			.deadband(0)
			.scaleTranslation(Constants.DriverConstants.TRANSLATION_SCALE)
			.allianceRelativeControl(true);
	

	// Choosers
	public static SendableChooser<Command> autoChooser;
	public static SendableChooser<DriveControlMode> modeChooser;

	public RobotContainer() {
		System.out.println("Configuring robot container");
		
		if (Robot.isSimulation()) {
			simulationSubsystem = new SimulationSubsystem();
		}

		configureChoosers();

		configureBindings();

		smartDashboardSubsystem.initalize();
	}

	private void configureChoosers() {
		autoChooser = AutoBuilder.buildAutoChooser();

		// Manually add the two calibrations to the auto chooser
		autoChooser.addOption("Calibrate Shoulder", new AutoCalibrateShoulderCommand());
		autoChooser.addOption("Calibrate Elevator", new AutoCalibrateElevatorCommand());
		autoChooser.addOption("Motor Coast", new AutoCoastCommand());

		modeChooser = new SendableChooser<>();
		modeChooser.setDefaultOption("Normal", DriveControlMode.NORMAL);
		modeChooser.addOption("Baby", DriveControlMode.BABY);
		SmartDashboard.putData("Drive Control Mode", modeChooser);

		System.out.println("Setup choosers");
	}

	public static BooleanSupplier isNormalModeSupplier() {
		return new BooleanSupplier() {
			@Override
			public boolean getAsBoolean() {
				return RobotContainer.driveControlMode == DriveControlMode.NORMAL;
			}
		};
	}

	public static boolean isNormalMode() {
		return RobotContainer.driveControlMode == DriveControlMode.NORMAL;
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
		* Intake coral
		*/
		driverController.rightTrigger(0.25).and(isNormalModeSupplier()).whileTrue(endEffectorSubsystem.setIntakeSpeedCommand(Constants.DriverConstants.INTAKE_SPEED));
		driverController.leftTrigger(0.25).and(isNormalModeSupplier()).whileTrue(endEffectorSubsystem.setIntakeSpeedCommand(Constants.DriverConstants.OUTTAKE_SPEED));


		// driverController.button(7).onTrue(new DemoEndEffector());

		/* 
		* Auto score coral on the reef
		*/
		driverController.multiButtonTrigger(4, 5).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(false, 0, true));
		driverController.multiButtonTrigger(2, 5).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(false, 1, true));
		driverController.multiButtonTrigger(1, 5).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(false, 2, true));
		driverController.multiButtonTrigger(3, 5).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(false, 3, true));

		driverController.multiButtonTrigger(4, 6).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(true, 0, true));
		driverController.multiButtonTrigger(2, 6).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(true, 1, true));
		driverController.multiButtonTrigger(1, 6).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(true, 2, true));
		driverController.multiButtonTrigger(3, 6).and(isNormalModeSupplier()).onTrue(new AutoScoreCoralCommand(true, 3, true));

		/*
		* Ground pickup setpoint
		*/
		driverController.button(9).and(isNormalModeSupplier()).onTrue(new GroundPickupConfigurationCommand());

		/*
		* Auto coral station pickup
		*/
		driverController.button(10).and(isNormalModeSupplier()).onTrue(new AutoPickupCoralStationCommand(true));

		/*
		* Climb commands
		*/
		driverController.button(7).and(isNormalModeSupplier()).whileTrue(RobotContainer.climbSubsystem.climbUp(Constants.DriverConstants.CLIMB_UP_SPEED));

		/*
		* Zero gyro
		*/
		driverController.button(8).and(isNormalModeSupplier()).onTrue(new InstantCommand(() -> RobotContainer.swerveSubsystem.zeroGyro(), RobotContainer.swerveSubsystem));

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
		return autoChooser.getSelected();
	}
}
