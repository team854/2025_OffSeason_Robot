package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class WristSubsystem extends SubsystemBase {
    /*
     * Motor
     */
    private final SparkMax wristMotor = new SparkMax(Constants.ArmConstants.Wrist.ID, MotorType.kBrushless);
    private final RelativeEncoder wristMotorEncoder = wristMotor.getEncoder();
    private SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    private double wristMotorTargetVoltage = 0;

    /*
     * Control
     */
    private final ProfiledPIDController wristController;
    private final SimpleMotorFeedforward wristFeedForward;
    private double maxControlAngularVelocity = 10000;

    /*
	 * Calibration
	 */
	private boolean calibrationEnabled = false;

    /*
     * Simulation
     */
    private final DCMotor wristGearbox = DCMotor.getNEO(1);
    private final SparkMaxSim wristMotorSim = new SparkMaxSim(wristMotor, wristGearbox);
    private DCMotorSim wristDcMotorSim;

    public WristSubsystem() {
        /*
         * Configure motor
         */

        System.out.println("Configuring wrist motor");

        this.wristMotorConfig.idleMode(IdleMode.kBrake);
        this.wristMotorConfig.inverted(false);

        // This sets all the conversions of the encoders so they automaticly convert from motor space to wrist space
        // It needs to be the reciprocal because the factors are multiplied with the encoder value
        this.wristMotorConfig.encoder.positionConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);
        this.wristMotorConfig.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);

        wristMotor.configure(this.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Configure PIDS
         */

        // Define the trapizoid profile for the wrist pid
        TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(
                Constants.ArmConstants.Wrist.MAX_VELOCITY, Constants.ArmConstants.Wrist.MAX_ACCELERATION);

        // Initalize the wrist motor pid
        wristController = new ProfiledPIDController(
                Constants.ArmConstants.Wrist.P,
                Constants.ArmConstants.Wrist.I,
                Constants.ArmConstants.Wrist.D,
                wristConstraints);
        wristController.enableContinuousInput(-180, 180);
        // Initalize the wrist motor feed forward
        wristFeedForward = new SimpleMotorFeedforward(
                Constants.ArmConstants.Wrist.S.in(Volt),
                Constants.ArmConstants.Wrist.V.in(Volt),
                Constants.ArmConstants.Wrist.A.in(Volt));

        /*
         * Initalize simulation
         */
        if (Robot.isSimulation()) {
            System.out.println("Creating wrist simulation");

            wristDcMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(wristGearbox,
                            Constants.ArmConstants.Wrist.MOI,
                            Constants.ArmConstants.Wrist.GEAR_RATIO),
                    wristGearbox,
                    0.02,
                    0);
        }

        System.out.println("Created WristSubsystem");
    }

    public SparkMaxConfig getCurrentWristConfig() {
		return this.wristMotorConfig;
	}

	public void setWristMotorConfig(SparkMaxConfig sparkMaxConfig) {
		this.wristMotorConfig = sparkMaxConfig;
		this.wristMotor.configure(this.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

    /**
     * 
     * @return The current rotation of the wrist
     */
    public Angle getWristAngle() {
        return Rotation.of(wristMotorEncoder.getPosition());
    }

    /**
     * 
     * @return The current setpoint of the wrist
     */
    public Angle getWristSetpoint() {
        // Gets the goal of the wrist controller (goal behaves better then setpoint for some reason)
        return Degree.of(wristController.getGoal().position);
    }

    /**
     * Sets the wrist setpoint
     * 
     * @param angle The target angle of the wrist
     */
    public void setWristSetpoint(Angle angle) {
        wristController.setGoal(angle.in(Degree));
    }

    /**
     * 
     * @return The angular velocity of the wrist
     */
    public AngularVelocity getWristVelocity() {
        // Gets the angular velocity in RPM and converts it to RPS
        return RotationsPerSecond.of(wristMotorEncoder.getVelocity() / 60);
    }

    /**
	 * Sets if the calibration mode is enabled for the shoulder. The calibration mode disables the pid.
	 * 
	 * @param state The target state of the calibration mode
	 */
	public void setCalibrationEnabled(boolean state) {
		this.calibrationEnabled = state;
		setWristMotorVoltage(Volt.of(0));

		if (this.calibrationEnabled == false) {
			resetWristSetpoint();
		}
	}

    public void setWristMotorVoltage(Voltage voltage) {
		this.wristMotorTargetVoltage = MathUtil.clamp(voltage.in(Volt), -4, 4);
		wristMotor.setVoltage(this.wristMotorTargetVoltage);
	}

    /**
     * Sets the wrist setpoint to the wrist's current angle
     */
    public void resetWristSetpoint() {
        setWristSetpoint(getWristAngle());
    }

    public Command setWristAngleCommand(Angle angle) {
        return runOnce(() -> setWristSetpoint(angle));
    }

    public Command gotoWristAngleCommand(Angle angle) {
		return new FunctionalCommand(
			() -> setWristSetpoint(angle),
			() -> {},
			(interrupted) -> {},
			() -> getWristAngle().isNear(getWristSetpoint(), Constants.ArmConstants.Wrist.TOLLERANCE),
			this
		).withTimeout(7);
	}

    public Command setWristSpeedCommand(AngularVelocity wristSpeed) {
        return new RunCommand(
                () -> {
                    // Limit the max commanded rotational speed
		            double wristDegreesPerSecond = MathUtil.clamp(wristSpeed.in(DegreesPerSecond), -this.maxControlAngularVelocity, this.maxControlAngularVelocity);
                    setWristSetpoint(getWristSetpoint().plus(Degree.of(wristDegreesPerSecond / 50)));
                }, this);
    }

    public void setMaxControlAngularVelocity(AngularVelocity angularVelocity) {
		this.maxControlAngularVelocity = angularVelocity.in(DegreesPerSecond);
	}

    @Override
    public void simulationPeriodic() {
        // Simulate wrist
        wristDcMotorSim.setInput(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        wristDcMotorSim.update(0.02);

        // Iterate on the wrist motor simulation
        wristMotorSim.iterate(wristDcMotorSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        // Add the current voltage as a load to the battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristDcMotorSim.getCurrentDrawAmps()));
    }

    @Override
    public void periodic() {
        if (this.calibrationEnabled) {
            return;
        }

        // Run the pid and feed forward for the wrist
        double wristVoltsOutput = wristController.calculate(getWristAngle().in(Degree))
                        + wristFeedForward.calculateWithVelocities(
                            getWristVelocity().in(RadiansPerSecond),
                            Units.degreesToRadians(wristController.getSetpoint().velocity));
        setWristMotorVoltage(Volt.of(wristVoltsOutput));
    }
}
