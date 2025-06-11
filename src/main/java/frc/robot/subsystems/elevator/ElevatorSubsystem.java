package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.simulation.ElevatorSimulation;
import frc.robot.utilities.math.EncoderUtilities;
import frc.robot.utilities.math.PoseUtilities;
import frc.robot.utilities.saftey.ArmSafteyUtilities;

// Credit to https://www.youtube.com/watch?v=_2fPVYDrq_E for the great tutorial

public class ElevatorSubsystem extends SubsystemBase {
    /*
     * Constants
     */
    private final double stage1MaxHeight = Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT.in(Meter);
    private final double stage2MaxHeight = Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT.in(Meter);
    private final double maxHeight = stage1MaxHeight + stage2MaxHeight;

    /*
     * Motors
     */
    private final SparkMax stage1Motor = new SparkMax(Constants.ElevatorConstants.Stage1.ID, MotorType.kBrushless);
    private final RelativeEncoder stage1Encoder = stage1Motor.getEncoder();
    private SparkMaxConfig stage1Config = new SparkMaxConfig();
    private double stage1MotorTargetVoltage = 0;

    private final SparkMax stage2Motor = new SparkMax(Constants.ElevatorConstants.Stage2.ID, MotorType.kBrushless);
    private final RelativeEncoder stage2Encoder = stage2Motor.getEncoder();
    private SparkMaxConfig stage2Config = new SparkMaxConfig();
    private double stage2MotorTargetVoltage = 0;

    /*
     * Control
     */
    private final ProfiledPIDController stage1Controller;
    private final ElevatorFeedforward stage1FeedForward;
    private Distance stage1MinHeight = Meter.of(0);
    private Distance stage1TargetHeight = Meter.of(0);

    private final ProfiledPIDController stage2Controller;
    private final ElevatorFeedforward stage2FeedForward;
    private Distance stage2MinHeight = Meter.of(0);
    private Distance stage2TargetHeight = Meter.of(0);

    /*
	 * Calibration
	 */
	private boolean calibrationEnabled = false;
    private double stage2Offset = (Constants.SimulationConstants.SIMULATE_ELEVATOR_OFFSET || Robot.isReal()) ? Constants.ElevatorConstants.Stage2.HEIGHT_OFFSET.in(Meter) : 0;

    /*
     * Simulation
     */
    private final DCMotor stage1Gearbox = DCMotor.getNEO(1);
    private final SparkMaxSim stage1MotorSim = new SparkMaxSim(stage1Motor, stage1Gearbox);
    private ElevatorSimulation stage1ElevatorSim = null;

    private final DCMotor stage2Gearbox = DCMotor.getNEO(1);
    private final SparkMaxSim stage2MotorSim = new SparkMaxSim(stage2Motor, stage2Gearbox);
    private ElevatorSimulation stage2ElevatorSim = null;

    public ElevatorSubsystem() {

        /*
         * Configure motors
         */

        System.out.println("Configuring elevator motors");

        // Configure stage 1 and save it to the motor
        this.stage1Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        this.stage1Config.inverted(true);

        // Configure the encoder so they automaticlly convert the motors rotation to meters
        double stage1Conversion = (Constants.ElevatorConstants.Stage1.DRUM_RADIUS.in(Meter) * 2 * Math.PI)
                / Constants.ElevatorConstants.Stage1.GEAR_RATIO;
        this.stage1Config.encoder.positionConversionFactor(stage1Conversion);
        this.stage1Config.encoder.velocityConversionFactor(stage1Conversion);
        stage1Motor.configure(this.stage1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        stage1Encoder.setPosition(0);
        

        // Configure stage 2 and save it to the motor
        this.stage2Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        this.stage2Config.inverted(true);
        this.stage2Config.absoluteEncoder.inverted(true);

        // Configure the encoder so they automaticlly convert the motors rotation to meters
        double stage2Conversion = (Constants.ElevatorConstants.Stage2.DRUM_RADIUS.in(Meter) * 2 * Math.PI)
                / Constants.ElevatorConstants.Stage2.GEAR_RATIO;
        double stage2AbsoluteConversion = (Constants.ElevatorConstants.Stage2.DRUM_RADIUS.in(Meter) * 2 * Math.PI)
                / Constants.ElevatorConstants.Stage2.ABSOLUTE_ENCODER_GEAR_RATIO;
        this.stage2Config.encoder.positionConversionFactor(stage2Conversion);
        this.stage2Config.encoder.velocityConversionFactor(stage2Conversion);
        this.stage2Config.absoluteEncoder.positionConversionFactor(stage2AbsoluteConversion);
        this.stage2Config.absoluteEncoder.velocityConversionFactor(stage2AbsoluteConversion);
        stage2Motor.configure(this.stage2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Automaticly set the offset of the relative encoder
        stage2Encoder.setPosition(stage2Motor.getAbsoluteEncoder().getPosition());

        /*
         * Configure PIDS
         */

        // Define the trapizoid profile for the pids
        TrapezoidProfile.Constraints stage1Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage1.MAX_VELOCITY,
                Constants.ElevatorConstants.Stage1.MAX_ACCELERATION);
        TrapezoidProfile.Constraints stage2Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage2.MAX_VELOCITY,
                Constants.ElevatorConstants.Stage2.MAX_ACCELERATION);

        // Initalize the motor pids
        stage1Controller = new ProfiledPIDController(
                Constants.ElevatorConstants.Stage1.P,
                Constants.ElevatorConstants.Stage1.I,
                Constants.ElevatorConstants.Stage1.D,
                stage1Constraints);
        stage2Controller = new ProfiledPIDController(
                Constants.ElevatorConstants.Stage2.P,
                Constants.ElevatorConstants.Stage2.I,
                Constants.ElevatorConstants.Stage2.D,
                stage2Constraints);

        // Configure feed forward
        stage1FeedForward = new ElevatorFeedforward(
                Constants.ElevatorConstants.Stage1.S.in(Volt),
                Constants.ElevatorConstants.Stage1.G.in(Volt),
                Constants.ElevatorConstants.Stage1.V.in(Volt),
                Constants.ElevatorConstants.Stage1.A.in(Volt));
        stage2FeedForward = new ElevatorFeedforward(
                Constants.ElevatorConstants.Stage2.S.in(Volt),
                Constants.ElevatorConstants.Stage2.G.in(Volt),
                Constants.ElevatorConstants.Stage2.V.in(Volt),
                Constants.ElevatorConstants.Stage2.A.in(Volt));

        /*
         * Initalize simulation
         */
        if (Robot.isSimulation()) {
            System.out.println("Creating elevator simulations");
            stage1ElevatorSim = new ElevatorSimulation(
                    stage1Gearbox,
                    Constants.ElevatorConstants.Stage1.GEAR_RATIO,
                    Constants.ElevatorConstants.Stage1.MASS,
                    Constants.ElevatorConstants.Stage1.DRUM_RADIUS,
                    Meter.of(0),
                    Meter.of(this.stage1MaxHeight),
                    true,
                    Meter.of(0),
                    0.02,
                    0);

            stage2ElevatorSim = new ElevatorSimulation(
                    stage2Gearbox,
                    Constants.ElevatorConstants.Stage2.GEAR_RATIO,
                    Constants.ElevatorConstants.Stage2.MASS,
                    Constants.ElevatorConstants.Stage2.DRUM_RADIUS,
                    Meter.of(0),
                    Meter.of(this.stage2MaxHeight),
                    true,
                    Meter.of(0),
                    0.02,
                    0);
        }

        System.out.println("Created ElevatorSubsystem");
    }

    public SparkMaxConfig getCurrentStage1Config() {
		return this.stage1Config;
	}

    public SparkMaxConfig getCurrentStage2Config() {
		return this.stage2Config;
	}

	public void setStage1MotorConfig(SparkMaxConfig sparkMaxConfig) {
		this.stage1Config = sparkMaxConfig;
		this.stage1Motor.configure(this.stage1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

    public void setStage2MotorConfig(SparkMaxConfig sparkMaxConfig) {
		this.stage2Config = sparkMaxConfig;
		this.stage2Motor.configure(this.stage2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

    public Distance getStage1Setpoint() {
        return Meter.of(stage1Controller.getGoal().position);
    }

    public Distance getStage2Setpoint() {
        return Meter.of(stage2Controller.getGoal().position);
    }

	public Distance getOverallSetpoint() {
		return getStage1Setpoint().plus(getStage2Height()).plus(getPivotPointOffset(true));
	}

    public void setStage1Setpoint(Distance height) {
        this.stage1TargetHeight = height.copy();
        // stage1Controller.setGoal(
        //        MathUtil.clamp(height.in(Meter), Math.max(this.stage1MinHeight.in(Meter), 0.02), this.stage1MaxHeight));
    }

    public void setStage2Setpoint(Distance height) {
        this.stage2TargetHeight = height.copy();
        // stage2Controller.setGoal(
        //        MathUtil.clamp(height.in(Meter), Math.max(this.stage2MinHeight.in(Meter), 0.02), this.stage2MaxHeight));
    }

    /**
     * 
     * @return The current height of stage 1 relative to its lowest position
     */
    public Distance getStage1Height() {
        return Meter.of(stage1Encoder.getPosition());
    }

    /**
     * 
     * @return The current height of stage 2 relative to its lowest position
     */
    public Distance getStage2Height() {
        return Meter.of(stage2Encoder.getPosition()).plus(Meter.of(stage2Offset));
    }

    public LinearVelocity getStage1HeightVelocity() {
        return MetersPerSecond.of(stage1Encoder.getVelocity() / 60);
    }

    public LinearVelocity getStage2HeightVelocity() {
        return MetersPerSecond.of(stage2Encoder.getVelocity() / 60);
    }

    /**
     * Sets the target overall height to the stages current overall height
     */
    public void resetStagesSetpoint() {
        setOverallHeight(getOverallHeight());
    }

    /**
     * Gets the offset of the shoulder pivot point from the ground
     * 
     * @param includeGroundHeight If the distance from the bot to the ground should be included
     * @return The pivot points offset from the ground / body pan in meters
     */
    public Distance getPivotPointOffset(boolean includeGroundHeight) {
        return Meter.of(((includeGroundHeight) ? Constants.RobotKinematicConstants.HEIGHT_OFF_GROUND.in(Meter) : 0)
                + Constants.ElevatorConstants.ZERO_HEIGHTS_ABOVE_BASE.in(Meter)
                + Constants.ArmConstants.Shoulder.STAGE_OFFSET_UP.in(Meter));
    }

    /**
     * Gets the height of both elevator stages relative to the carpet
     * 
     * @return The height of the elevator relative to the carpet in meters
     */
    public Distance getOverallHeight() {
        return getStage1Height().plus(getStage2Height()).plus(getPivotPointOffset(true));
    }

    /**
     * Checks if a target height is reachable by the elevator
     * 
     * @param height The target height from the ground in meters
     * @return Whether the target height is posible
     */
    public boolean checkOverallHeightPossible(Distance height) {

        // Because the elevator even at its lowest is a little bit off the ground its base height has to be subtracted from the overall height to get the local height
        double elevatorLocalHeight = (height.minus(getPivotPointOffset(true))).in(Meter);

        return elevatorLocalHeight >= 0 && elevatorLocalHeight <= this.maxHeight;
    }

    private Pair<Distance, Distance> calculateOverallHeight(Distance targetHeight) {
        // Because the target height is based on the distance from the carpet it has to be converted to elevator height
        double targetHeightMeters = targetHeight.in(Meter) - getPivotPointOffset(true).in(Meter);

        // Clamp the target height so it is within the possible range of the elevator
        targetHeightMeters = MathUtil.clamp(targetHeightMeters, 0, this.maxHeight);

        // Get the percent of the total height the elevator is going to
        double targetRatio = targetHeightMeters / maxHeight;

        double target1Height = MathUtil.clamp(
                this.stage1MaxHeight * targetRatio, 0.01,
                this.stage1MaxHeight);
        double target2Height = MathUtil.clamp(
                this.stage2MaxHeight * targetRatio, 0.01,
                this.stage2MaxHeight);

        return new Pair<Distance, Distance>(Meter.of(target1Height), Meter.of(target2Height));
    }

    /**
     * Sets the overall height of the elevator by moving both stages
     * 
     * @param targetHeight The target height of the elevator relative to the ground in meters
     */
    public void setOverallHeight(Distance targetHeight) {

        Pair<Distance, Distance> stage_heights = calculateOverallHeight(targetHeight);

        // Set each elevators setpoint to the calculated heights in meters
        setStage1Setpoint(stage_heights.getFirst());
        setStage2Setpoint(stage_heights.getSecond());
    }

	/**
     * Sets the overall height of the elevator by moving both stages
     * 
     * @param targetHeight The target height of the elevator relative to the ground in meters
     */
    public Command setOverallHeightCommand(Distance targetHeight) {
		return runOnce(() -> setOverallHeight(targetHeight));
	}

	public Command gotoOverallHeightCommand(Distance targetHeight) {
		return new FunctionalCommand(
			() -> setOverallHeight(targetHeight),
			() -> {},
			(interrupted) -> {},
			() -> getOverallHeight().isNear(getOverallSetpoint(), Constants.ElevatorConstants.TOLLERANCE),
			this
		).withTimeout(7);
	}

    public Distance getMaxHeight() {
        return Meter.of(this.maxHeight);
    }

    public Distance getMaxGroundHeight() {
        return getMaxHeight().plus(getPivotPointOffset(true));
    }

    /**
	 * Sets if the calibration mode is enabled for the elevator. The calibration mode disables the pid.
	 * 
	 * @param state The target state of the calibration mode
	 */
	public void setCalibrationEnabled(boolean state) {
		this.calibrationEnabled = state;

        setStage1MotorVoltage(Volt.of(0));
        setStage2MotorVoltage(Volt.of(0));
		if (this.calibrationEnabled == false) {
			RobotContainer.elevatorSubsystem.resetStagesSetpoint();
		}
	}

    public Command calibrateElevatorCommand(Consumer<Double> zeroOffsetConsumer) {
		List<Double> elevatorVelocityHistory = new ArrayList<>();
		return new FunctionalCommand(() -> {
			// Add some values to the encoder history so it has time to accelerate
			for (int i = 0; i < 5; i++) {
				elevatorVelocityHistory.add(1.0);
			}
			
			// Enable calibration mode and start the motor moving
			setCalibrationEnabled(true);
			setStage2MotorVoltage(Volt.of(1.5));
		}, () -> {
			// Remove the oldest velocity from the start and add the current velocity to the end
			elevatorVelocityHistory.remove(0);
			elevatorVelocityHistory.add(getStage2HeightVelocity().in(MetersPerSecond));
		}, (interrupted) -> {
			// Calculate the zero offset
			Distance outputZeroOffset = Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT.minus(getStage2Height()).minus(Constants.ElevatorConstants.Stage2.HEIGHT_OFFSET);

			// Update the consumer with the zero offset
			zeroOffsetConsumer.accept(outputZeroOffset.in(Meter));

            // Only update the offset if offsets should be appled in simulation or its a real robot
            if (Constants.SimulationConstants.SIMULATE_ELEVATOR_OFFSET || Robot.isReal()) {
                // Update the local offset until it can be manually updated in the constants file
                this.stage2Offset = outputZeroOffset.in(Meter);
            }

			// Disable calibration mode
			setCalibrationEnabled(false);
		}, () -> {
			// Get the average velocity
			double velocityAverage = 0;
			for (double velocity : elevatorVelocityHistory) {
				velocityAverage+=velocity;
			}
			velocityAverage /= elevatorVelocityHistory.size();

			// If it is bellow 0.01 meter per second finish the command
			return Math.abs(velocityAverage) < 0.01;
		}, this);
	}

    public void setStage1MotorVoltage(Voltage voltage) {
		this.stage1MotorTargetVoltage = voltage.in(Volt);
		stage1Motor.setVoltage(MathUtil.clamp(voltage.in(Volt), -10, 10));
	}

    public void setStage2MotorVoltage(Voltage voltage) {
		this.stage2MotorTargetVoltage = voltage.in(Volt);
		stage2Motor.setVoltage(MathUtil.clamp(voltage.in(Volt), -10, 10));
	}

    public Voltage getStage1MotorVoltage() {
		return Volt.of(this.stage1MotorTargetVoltage);
	}

    public Voltage getStage2MotorVoltage() {
		return Volt.of(this.stage2MotorTargetVoltage);
	}

    public ElevatorSimulation getStage1ElevatorSim() {
        return this.stage1ElevatorSim;
    }

    public ElevatorSimulation getStage2ElevatorSim() {
        return this.stage2ElevatorSim;
    }

    @Override
    public void simulationPeriodic() {

        // Update the weight of stage 1 depending on whether it has coral
		Mass stage1_offset_weight = RobotContainer.endEffectorSubsystem.hasCoral() ? Constants.SimulationConstants.CORAL_WEIGHT : Kilogram.of(0);
		stage1ElevatorSim.setWeight(Constants.ElevatorConstants.Stage1.MASS.plus(stage1_offset_weight));

        // Update the weight of stage 2 depending on whether it has coral
		Mass stage2_offset_weight = RobotContainer.endEffectorSubsystem.hasCoral() ? Constants.SimulationConstants.CORAL_WEIGHT : Kilogram.of(0);
		stage2ElevatorSim.setWeight(Constants.ElevatorConstants.Stage2.MASS.plus(stage2_offset_weight));

        // Update the voltage of the motor
        stage1ElevatorSim.setInputVoltage(Volt.of(stage1MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()));
        stage2ElevatorSim.setInputVoltage(Volt.of(stage2MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()));

        // Update every 20 miliseconds
        stage1ElevatorSim.update(0.02);
        stage2ElevatorSim.update(0.02);

        // Iterate the simulation of both motors
        stage1MotorSim.iterate(stage1ElevatorSim.getVelocity().in(MetersPerSecond) * 60,
                RoboRioSim.getVInVoltage(), 0.02);

        stage2MotorSim.iterate(stage2ElevatorSim.getVelocity().in(MetersPerSecond) * 60,
                RoboRioSim.getVInVoltage(), 0.02);
        
        // Add the current voltage as a load to the battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                stage1ElevatorSim.getCurrentDrawAmps().in(Amp) + stage2ElevatorSim.getCurrentDrawAmps().in(Amp)));

    }

    private Distance getElevatorMinimumHeight(Pose3d robotPose, Angle shoulderSetpointAngle, Distance elevatorHeightSetpoint) {

        // Calculate the end effector pose once the arm reaches its targets
        Pose3d endEffectorPose = RobotContainer.endEffectorSubsystem.calculateEndEffectorPose(robotPose, elevatorHeightSetpoint, shoulderSetpointAngle, RobotContainer.wristSubsystem.getWristAngle());
        
        // Get the offset bettween the end effector and the center of the robot
        Translation3d effectorOffset = endEffectorPose.getTranslation().minus(robotPose.getTranslation()).rotateBy(robotPose.getRotation().unaryMinus());

        // Get the minimum elevator height at that offset on the arm
        double minimumHeight = ArmSafteyUtilities.getMinimumEndEffectorHeight(effectorOffset.getMeasureX()).in(Meter);

        // Calculate the minium height of the elevator by calculating the height of the elevator when at the set angle then adding the minimum height if the end effector
        double elevatorMinimumHeight = (-Math.sin(shoulderSetpointAngle.in(Radian)) * Constants.ArmConstants.LENGTH.in(Meter)) + minimumHeight;
        

        return Meter.of(elevatorMinimumHeight);
    }


    @Override
    public void periodic() {
        if (this.calibrationEnabled) {
			return;
		}

        if (!Constants.ElevatorConstants.ENABLED) {
            return;
        }

        // Get the robot pose, overall elevator height, and shoulder angle
        Pose3d robotPose = new Pose3d(RobotContainer.swerveSubsystem.getPose());
        
        // Look ahead 1 second to help compensate for the time required to move the elevator
        AngularVelocity currentShoulderSetpointVelocity = RobotContainer.shoulderSubsystem.getShoulderSetpointVelocity();
        Angle currentShoulderSetpoint = RobotContainer.shoulderSubsystem.getShoulderSetpoint();
        Angle currentShoulderSetpointLookAhead = currentShoulderSetpoint.plus(Degree.of(currentShoulderSetpointVelocity.in(DegreesPerSecond))).times(0.2);
        
        Angle currentShoulderAngle = RobotContainer.shoulderSubsystem.getShoulderAngle();

        // Get the overall hieght of the elevator setpoint
        Distance currentOverallSetpoint = getOverallSetpoint();

        // Compute it twice with and without the look ahead and use which ever is higher so its safer
        double elevatorMinimumHeight = Math.max(Math.max(getElevatorMinimumHeight(robotPose, currentShoulderSetpoint, currentOverallSetpoint).in(Meter), getElevatorMinimumHeight(robotPose, currentShoulderAngle, currentOverallSetpoint).in(Meter)), 
                                                getElevatorMinimumHeight(robotPose, currentShoulderSetpointLookAhead, currentOverallSetpoint).in(Meter));
        
        // If the shoulder is pitching up then lower the threshold a bit to prevent sticking
        double minHeightOffset = (currentShoulderSetpointVelocity.in(DegreesPerSecond) > 0.05) ? 0.5 : 0; FIGNRE ME OUT OK BYE

        Pair<Distance, Distance> elevatorStagesMinimum = calculateOverallHeight(Meter.of(elevatorMinimumHeight - minHeightOffset));
        this.stage1MinHeight = elevatorStagesMinimum.getFirst();
        this.stage2MinHeight = elevatorStagesMinimum.getSecond();

        stage1Controller.setGoal(
                MathUtil.clamp(this.stage1TargetHeight.in(Meter), Math.max(this.stage1MinHeight.in(Meter), 0.02), this.stage1MaxHeight));

        stage2Controller.setGoal(
                MathUtil.clamp(this.stage2TargetHeight.in(Meter), Math.max(this.stage2MinHeight.in(Meter), 0.04), this.stage2MaxHeight));

        double stage1VoltsOutput = MathUtil.clamp(
                stage1Controller.calculate(getStage1Height().in(Meter)) + stage1FeedForward
                        .calculateWithVelocities(getStage1HeightVelocity().in(MetersPerSecond),
                                stage1Controller.getSetpoint().velocity),
                -10, 10);

        setStage1MotorVoltage(Volt.of(stage1VoltsOutput));

        double stage2VoltsOutput = MathUtil.clamp(
                stage2Controller.calculate(getStage2Height().in(Meter)) + stage2FeedForward
                        .calculateWithVelocities(getStage2HeightVelocity().in(MetersPerSecond),
                                stage2Controller.getSetpoint().velocity),
                -10, 10);

        setStage2MotorVoltage(Volt.of(stage2VoltsOutput));
    }
}
