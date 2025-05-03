package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.simulation.ShoulderSimulation;
import frc.robot.utilities.math.EncoderUtilities;

public class ShoulderSubsystem extends SubsystemBase {
	/*
	 * Constants
	 */
	// This value gets added on to the absolute encoder position to correct for the push back and to bring 0 to horizontal
	private final double shoulderPushBackHorizontal = Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_PUSH_BACK
			.in(Degree) - Constants.ArmConstants.Shoulder.MIN_ANGLE.in(Degree);
	private final double shoulderMaxAngle = Constants.ArmConstants.Shoulder.MAX_ANGLE.in(Degree);
	private final double shoulderMinAngle = Constants.ArmConstants.Shoulder.MIN_ANGLE.in(Degree);
	/*
	 * Motor
	 */
	private final SparkMax shoulderMotor = new SparkMax(Constants.ArmConstants.Shoulder.ID, MotorType.kBrushless);
	private final RelativeEncoder shoulderMotorEncoder = shoulderMotor.getEncoder();
	private final AbsoluteEncoder shoulderMotorAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();
	private double shoulderMotorTargetVoltage = 0;

	/*
	 * Control
	 */
	private final ProfiledPIDController shoulderController;
	private final ArmFeedforward shoulderFeedForward;

	/*
	 * Calibration
	 */
	private boolean calibrationEnabled = false;

	/*
	 * Simulation
	 */
	private final DCMotor shoulderGearbox = DCMotor.getNEO(1);
	private final SparkMaxSim shoulderMotorSim = new SparkMaxSim(shoulderMotor, shoulderGearbox);
	private ShoulderSimulation shoulderArmSim = null;
	private double lastElevatorVelocity = 0;
	private double lastForwardVelocity = 0;

	public ShoulderSubsystem() {
		/*
		 * Configure motor
		 */

		System.out.println("Configuring shoulder motor");

		SparkMaxConfig shoulderMotorConfig = new SparkMaxConfig();
		shoulderMotorConfig.idleMode(IdleMode.kBrake); // Brake so the arm doesn't fall
		shoulderMotorConfig.inverted(false);
		shoulderMotorConfig.absoluteEncoder.inverted(false);

		

		// This sets all the conversions of the encoders so they automaticly convert from motor space to arm space
		// It needs to be the reciprocal because the factors are multiplied with the encoder value
		shoulderMotorConfig.encoder.positionConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
		shoulderMotorConfig.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder
				.positionConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder
				.velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder.zeroOffset(getZeroOffset(false));

		shoulderMotor.configure(shoulderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		/*
		 * Configure PIDS
		 */

		// Define the trapizoid profile for the shoulder pid
		TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(
				Constants.ArmConstants.Shoulder.MAX_VELOCITY, Constants.ArmConstants.Shoulder.MAX_ACCELERATION);

		// Initalize the shoulder motor pid
		shoulderController = new ProfiledPIDController(
				Constants.ArmConstants.Shoulder.P,
				Constants.ArmConstants.Shoulder.I,
				Constants.ArmConstants.Shoulder.D,
				shoulderConstraints);
		// Initalize the shoulder motor feed forward
		shoulderFeedForward = new ArmFeedforward(
				Constants.ArmConstants.Shoulder.S.in(Volt),
				Constants.ArmConstants.Shoulder.G.in(Volt),
				Constants.ArmConstants.Shoulder.V.in(Volt),
				Constants.ArmConstants.Shoulder.A.in(Volt));

		/*
		 * Initalize simulation
		 */
		if (Robot.isSimulation()) {
			System.out.println("Creating shoulder simulation");

			shoulderArmSim = new ShoulderSimulation(shoulderGearbox, Constants.ArmConstants.Shoulder.GEAR_RATIO,
					Constants.ArmConstants.Shoulder.MASS,
					Constants.ArmConstants.LENGTH,
					Constants.ArmConstants.Shoulder.MIN_ANGLE,
					Constants.ArmConstants.Shoulder.MAX_ANGLE,
					true,
					Radian.of(0.0),
					0.02,
					0.0);
		}

		System.out.println("Created ShoulderSubsystem");
	}

	private double getZeroOffset(boolean armSpace) {
		// This calculates the zero offset of the absolute encoder
		// It gets the offset from the config and adds a push back angle onto that
		// Adding a push back angle prevents the code from having to deal with the posibility of the encoder going over it's zero point
		// This does assume that the arm is zeroed so the claw is facing down
		double realOffset = Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_OFFSET.in(Rotation) + (shoulderPushBackHorizontal / 360);

		if (!armSpace) {
			realOffset *= Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO;
		}

		// The absolute encoder only takes in values from 0 to 1 so if its less then 0 it needs to be brought back into the 0 to 1 range
		realOffset = EncoderUtilities.clampAbsoluteEncoder(Rotation.of(realOffset)).in(Rotation);
		
		return realOffset;
	}

	/**
	 * 
	 * @return The current rotation of the shoulder with 0 being parallel to the ground
	 */
	public Angle getShoulderAngle() {
		// Gets the current shoulder angle in rotations then converts it to degrees and subtracts the shoulder push back value
		return Rotation.of(shoulderMotorAbsoluteEncoder.getPosition()).minus(Degree.of(shoulderPushBackHorizontal));
	}

	/**
	 * 
	 * @return The current setpoint of the shoulder with 0 being parallel to the ground
	 */
	public Angle getShoulderSetpoint() {
		// Gets the goal of the shoulder controller (goal behaves better then setpoint for some reason)
		return Degree.of(shoulderController.getGoal().position);
	}

	/**
	 * 
	 * @return The current angular velocity of the shoulder setpoint
	 */
	public AngularVelocity getShoulderSetpointVelocity() {
		// Gets the goal of the shoulder controller (goal behaves better then setpoint for some reason)
		return DegreesPerSecond.of(shoulderController.getGoal().velocity);
	}

	/**
	 * Sets the shoulder setpoint and ensures that it is within the range of motion of the shoulder
	 * 
	 * @param angle The target angle of the shoulder with 0 being parallel to the ground
	 */
	public void setShoulderSetpoint(Angle angle) {
		shoulderController.setGoal(MathUtil.clamp(angle.in(Degree), shoulderMinAngle, shoulderMaxAngle));
	}

	/**
	 * 
	 * @return The angular velocity of the shoulder
	 */
	public AngularVelocity getShoulderVelocity() {
		// Gets the angular velocity in RPM and converts it to RPS
		return RotationsPerSecond.of(shoulderMotorAbsoluteEncoder.getVelocity() / 60);
	}

	public Voltage getShoulderMotorVoltage() {
		return Volt.of(this.shoulderMotorTargetVoltage);
	}

	/**
	 * Sets the shoulder setpoint to the shoulder's current angle
	 */
	public void resetShoulderSetpoint() {
		Angle shoulderAngle = getShoulderAngle();
		shoulderController.reset(shoulderAngle.in(Degree));
		setShoulderSetpoint(shoulderAngle);
	}

	public Command setShoulderAngleCommand(Angle angle) {
		return runOnce(() -> setShoulderSetpoint(angle));
	}

	public Command gotoShoulderAngleCommand(Angle angle) {
		return new FunctionalCommand(
			() -> setShoulderSetpoint(angle),
			() -> {},
			(interrupted) -> {},
			() -> getShoulderAngle().isNear(getShoulderSetpoint(), Constants.ArmConstants.Shoulder.TOLLERANCE),
			this
		).withTimeout(7);
	}

	public Command setShoulderSpeedCommand(AngularVelocity shoulderSpeed) {
		return new RunCommand(() -> setShoulderSetpoint(
				getShoulderSetpoint().plus(Degree.of(shoulderSpeed.in(DegreesPerSecond) / 50))), this);
	}

	/**
	 * Sets if the calibration mode is enabled for the shoulder. The calibration mode disables the pid.
	 * 
	 * @param state The target state of the calibration mode
	 */
	public void setCalibrationEnabled(boolean state) {
		this.calibrationEnabled = state;

		if (this.calibrationEnabled == true) {
			setShoulderMotorVoltage(Volt.of(0));
		} else {
			resetShoulderSetpoint();
		}
	}

	public void setShoulderMotorVoltage(Voltage voltage) {
		this.shoulderMotorTargetVoltage = voltage.in(Volt);
		shoulderMotor.setVoltage(MathUtil.clamp(voltage.in(Volt), -10, 10));
	}

	public Command calibrateShoulderCommand(Consumer<Double> zeroOffsetConsumer) {
		List<Double> encoderVelocityHistory = new ArrayList<>();
		return new FunctionalCommand(() -> {
			// Add some values to the encoder history so it has time to accelerate
			for (int i = 0; i < 5; i++) {
				encoderVelocityHistory.add(1.0);
			}
			
			// Enable calibration mode and start the motor moving
			setCalibrationEnabled(true);
			setShoulderMotorVoltage(Volt.of(0.8));
		}, () -> {
			// Remove the oldest velocity from the start and add the current velocity to the end
			encoderVelocityHistory.remove(0);
			encoderVelocityHistory.add(getShoulderVelocity().in(DegreesPerSecond));
		}, (interrupted) -> {
			// Calculate the zero offset
			Angle outputZeroOffset = EncoderUtilities.calculateZeroOffset(getShoulderAngle(), Constants.ArmConstants.Shoulder.MIN_ANGLE, Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_OFFSET);

			// Update the consumer with the zero offset
			zeroOffsetConsumer.accept(outputZeroOffset.in(Degree));

			// Disable calibration mode
			setCalibrationEnabled(false);
		}, () -> {
			// Get the average velocity
			double velocityAverage = 0;
			for (double velocity : encoderVelocityHistory) {
				velocityAverage+=velocity;
			}
			velocityAverage /= encoderVelocityHistory.size();

			// If it is bellow 0.25 degrees per second finish the command
			return Math.abs(velocityAverage) < 0.25;
		}, this);
	}


	@Override
	public void simulationPeriodic() {

		// Calculate the elevator velocity and acceleration
		double elevatorVelocity = RobotContainer.elevatorSubsystem.getStage1ElevatorSim().getVelocity().in(MetersPerSecond) + RobotContainer.elevatorSubsystem.getStage2ElevatorSim().getVelocity().in(MetersPerSecond);
		double elevatorAcceleration = (elevatorVelocity - this.lastElevatorVelocity) / 0.02;
		this.lastElevatorVelocity = elevatorVelocity;

		// Calculate the forward swerve velocity and acceleration
		Vector2 velocityVector = RobotContainer.swerveSubsystem.swerveDrive.getMapleSimDrive().get().getLinearVelocity();
		Pose2d roboPose = RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get();
		double forwardVelocity = new Translation2d(velocityVector.x, velocityVector.y).rotateBy(roboPose.getRotation().unaryMinus()).getX();
		double forwardAcceleration = (forwardVelocity - this.lastForwardVelocity) / 0.02;
		this.lastForwardVelocity = forwardVelocity;

		// Simulate shoulder
		shoulderArmSim.updatePivotVerticalAcceleration(MetersPerSecondPerSecond.of(elevatorAcceleration));
		shoulderArmSim.updatePivotForwardAcceleration(MetersPerSecondPerSecond.of(forwardAcceleration));

		// Update the weight of the shoulder depending on whether it has coral
		// Multiply it by two because the weight is far out on the arm so it helps to aproximate that
		Mass offset_weight = RobotContainer.endEffectorSubsystem.hasCoral() ? Constants.SimulationConstants.CORAL_WEIGHT.times(2) : Kilogram.of(0);
		shoulderArmSim.setWeight(Constants.ArmConstants.Shoulder.MASS.plus(offset_weight));

		shoulderArmSim.setInputVoltage(Volt.of(shoulderMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()));

		shoulderArmSim.update(0.02);

		// Iterate in the shoulder motor simulation
		shoulderMotorSim.iterate(shoulderArmSim.getVelocity().in(RotationsPerSecond) * 60,
				RoboRioSim.getVInVoltage(), 0.02);

		// The motor sim can't update both the absolute encoder and the relative encoder
		// To get around this the code just updates the absolute encoder for it
		SparkAbsoluteEncoderSim absoluteEncoderSim = shoulderMotorSim.getAbsoluteEncoderSim();
		
		// Set the position to the simulated position plus the zero offset in arm space
		absoluteEncoderSim
				.setPosition(shoulderMotorSim.getPosition() + getZeroOffset(true));
		absoluteEncoderSim.setVelocity(shoulderMotorSim.getVelocity());
		
		// Add the current voltage as a load to the battery
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
				shoulderArmSim.getCurrentDrawAmps().in(Amp))); // MAKE THE WEGIHT DYNAMIC AND FIGURE OUT HOW / WHAT FORCES SHOULD BE APPLIED TO THE ELEVATOR WHEN THE ARM MOVES
	}

	@Override
	public void periodic() {
		if (calibrationEnabled) {
			return;
		}

		// Run the pid and feed forward for the shoulder
		double shoulderVoltsOutput = shoulderController.calculate(getShoulderAngle().in(Degree))
						+ shoulderFeedForward.calculateWithVelocities(
								getShoulderAngle().in(Radian),
								getShoulderVelocity().in(RadiansPerSecond),
								Units.degreesToRadians(shoulderController.getGoal().velocity));
		
		
		setShoulderMotorVoltage(Volt.of(shoulderVoltsOutput));
	}
}
