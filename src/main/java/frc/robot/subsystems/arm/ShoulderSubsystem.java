package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShoulderSubsystem extends SubsystemBase {
	/*
	 * Constants
	 */
	// This value gets added on to the absolute encoder position to correct for the push back and to bring 0 to horizontal
	private final double shoulderPushBackHorizontal = Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_PUSH_BACK
			.in(Degree) + 90;
	private final double shoulderMaxAngle = Constants.ArmConstants.Shoulder.MAX_ANGLE.in(Degree);
	private final double shoulderMinAngle = Constants.ArmConstants.Shoulder.MIN_ANGLE.in(Degree);
	/*
	 * Motor
	 */
	private final SparkMax shoulderMotor = new SparkMax(Constants.ArmConstants.Shoulder.ID, MotorType.kBrushless);
	private final RelativeEncoder shoulderMotorEncoder = shoulderMotor.getEncoder();
	private final AbsoluteEncoder shoulderMotorAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();

	/*
	 * Control
	 */
	private final ProfiledPIDController shoulderController;
	private final ArmFeedforward shoulderFeedFoward;

	/*
	 * Simulation
	 */
	private final DCMotor shoulderGearbox = DCMotor.getNEO(1);
	private final SparkMaxSim shoulderMotorSim = new SparkMaxSim(shoulderMotor, shoulderGearbox);
	private SingleJointedArmSim shoulderArmSim = null;

	public ShoulderSubsystem() {
		/*
		 * Configure motor
		 */

		System.out.println("Configuring shoulder motor");

		SparkMaxConfig shoulderMotorConfig = new SparkMaxConfig();
		shoulderMotorConfig.idleMode(IdleMode.kBrake); // Brake so the arm doesn't fall
		shoulderMotorConfig.inverted(false);
		shoulderMotorConfig.absoluteEncoder.inverted(false);

		// This calculates the zero offset of the absolute encoder
		// It first converts the rotation from arm space to encoder space then adds a push back angle
		// Adding a push back angle prevents the code from having to deal with the posibility of the encoder going over it zero point
		// This does assume that the arm is zeroed so the intake is facing down
		double realOffset = ((Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_OFFSET.in(Rotation)
				* Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO)
				+ Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_PUSH_BACK.in(Rotation)) % 1;

		// The absolute encoder only takes in values from 0 to 1 so if its less then 0 it needs to be brought back into the 0 to 1 range
		if (realOffset < 0) {
			realOffset = 1 + realOffset;
		}

		// This sets all the conversions of the encoders so they automaticly convert from motor space to arm space
		// It needs to be the reciprocal because the factors are multiplied with the encoder value
		shoulderMotorConfig.encoder.positionConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
		shoulderMotorConfig.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder
				.positionConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder
				.velocityConversionFactor(1 / Constants.ArmConstants.Shoulder.ABSOLUTE_ENCODER_GEAR_RATIO);
		shoulderMotorConfig.absoluteEncoder.zeroOffset(realOffset);

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
		// Initalize the shoulder motor feed foward
		shoulderFeedFoward = new ArmFeedforward(
				Constants.ArmConstants.Shoulder.S.in(Volt),
				Constants.ArmConstants.Shoulder.G.in(Volt),
				Constants.ArmConstants.Shoulder.V.in(Volt),
				Constants.ArmConstants.Shoulder.A.in(Volt));

		/*
		 * Initalize simulation
		 */
		if (Robot.isSimulation()) {
			System.out.println("Creating shoulder simulation");

			shoulderArmSim = new SingleJointedArmSim(shoulderGearbox, Constants.ArmConstants.Shoulder.GEAR_RATIO,
					SingleJointedArmSim.estimateMOI(
							Constants.ArmConstants.LENGTH.in(Meter),
							Constants.ArmConstants.Shoulder.MASS.in(Kilogram)
									+ Constants.ArmConstants.Wrist.MASS.in(Kilogram)),
					Constants.ArmConstants.LENGTH.in(Meter),
					Constants.ArmConstants.Shoulder.MIN_ANGLE.in(Radian),
					Constants.ArmConstants.Shoulder.MAX_ANGLE.in(Radian),
					true,
					0,
					0.02,
					0);
		}

		System.out.println("Created ShoulderSubsystem");
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

	@Override
	public void simulationPeriodic() {
		// Simulate shoulder
		shoulderArmSim.setInput(shoulderMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

		shoulderArmSim.update(0.02);

		// Iterate in the shoulder motor simulation
		shoulderMotorSim.iterate(Units.radiansToRotations(shoulderArmSim.getVelocityRadPerSec()) * 60,
				RoboRioSim.getVInVoltage(), 0.02);

		// The motor sim can't update both the absolute encoder and the relative encoder
		// To get around this the code just updates the absolute encoder for it
		SparkAbsoluteEncoderSim absoluteEncoderSim = shoulderMotorSim.getAbsoluteEncoderSim();
		absoluteEncoderSim
				.setPosition(shoulderMotorSim.getPosition() + Units.degreesToRotations(shoulderPushBackHorizontal));
		absoluteEncoderSim.setVelocity(shoulderMotorSim.getVelocity());

		// Add the current voltage as a load to the battery
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
				shoulderArmSim.getCurrentDrawAmps()));
	}

	@Override
	public void periodic() {
		// Run the pid and feed foward for the shoulder
		double shoulderVoltsOutput = MathUtil
				.clamp(shoulderController.calculate(getShoulderAngle().in(Degree))
						+ shoulderFeedFoward.calculateWithVelocities(
								getShoulderAngle().in(Radian),
								getShoulderVelocity().in(RadiansPerSecond),
								Units.degreesToRadians(shoulderController.getGoal().velocity)),
						-10, 10);
		shoulderMotor.setVoltage(shoulderVoltsOutput);
	}
}
