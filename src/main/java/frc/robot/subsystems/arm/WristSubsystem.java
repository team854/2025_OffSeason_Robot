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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
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

    /*
     * Control
     */
    private final ProfiledPIDController wristController;
    private final SimpleMotorFeedforward wristFeedFoward;

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

        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig.idleMode(IdleMode.kBrake);
        wristMotorConfig.inverted(false);

        // This sets all the conversions of the encoders so they automaticly convert from motor space to wrist space
        // It needs to be the reciprocal because the factors are multiplied with the encoder value
        wristMotorConfig.encoder.positionConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);
        wristMotorConfig.encoder.velocityConversionFactor(1 / Constants.ArmConstants.Wrist.GEAR_RATIO);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        // Initalize the wrist motor feed foward
        wristFeedFoward = new SimpleMotorFeedforward(
                Constants.ArmConstants.Wrist.S.in(Volt),
                Constants.ArmConstants.Wrist.G.in(Volt),
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
     * Sets the wrist setpoint to the wrist's current angle
     */
    public void resetWristSetpoint() {
        setWristSetpoint(getWristAngle());
    }

    public Command setWristAngleCommand(Angle angle) {
        return runOnce(() -> setWristSetpoint(angle));
    }

    public Command setWristSpeedCommand(AngularVelocity wristSpeed) {
        return new RunCommand(
                () -> setWristSetpoint(getWristSetpoint().plus(Degree.of(wristSpeed.in(DegreesPerSecond) / 50))), this);
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
        // Run the pid and feed foward for the wrist
        double wristVoltsOutput = MathUtil
                .clamp(wristController.calculate(getWristAngle().in(Degree))
                        + wristFeedFoward.calculateWithVelocities(
                            getWristVelocity().in(RadiansPerSecond),
                            Units.degreesToRadians(wristController.getSetpoint().velocity)),
                    -10, 10);
        wristMotor.setVoltage(wristVoltsOutput);
    }
}
