package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

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

    private final SparkMax stage2Motor = new SparkMax(Constants.ElevatorConstants.Stage2.ID, MotorType.kBrushless);
    private final RelativeEncoder stage2Encoder = stage2Motor.getEncoder();

    /*
     * Control
     */
    private final ProfiledPIDController stage1Controller;
    private final ElevatorFeedforward stage1FeedFoward;

    private final ProfiledPIDController stage2Controller;
    private final ElevatorFeedforward stage2FeedFoward;

    /*
     * Simulation
     */
    private final DCMotor stage1Gearbox = DCMotor.getNEO(1);
    private final SparkMaxSim stage1MotorSim = new SparkMaxSim(stage1Motor, stage1Gearbox);
    private ElevatorSim stage1ElevatorSim = null;

    private final DCMotor stage2Gearbox = DCMotor.getNEO(1);
    private final SparkMaxSim stage2MotorSim = new SparkMaxSim(stage2Motor, stage2Gearbox);
    private ElevatorSim stage2ElevatorSim = null;

    public ElevatorSubsystem() {

        /*
         * Configure motors
         */

        System.out.println("Configuring elevator motors");

        // Configure stage 1 and save it to the motor
        SparkMaxConfig stage1Config = new SparkMaxConfig();
        stage1Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        stage1Config.inverted(false);

        // Configure the encoder so they automaticlly convert the motors rotation to meters
        double stage1Conversion = (Constants.ElevatorConstants.Stage1.DRUM_RADIUS.in(Meter) * 2 * Math.PI)
                / Constants.ElevatorConstants.Stage1.GEAR_RATIO;
        stage1Config.encoder.positionConversionFactor(stage1Conversion);
        stage1Config.encoder.velocityConversionFactor(stage1Conversion);
        stage1Motor.configure(stage1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure stage 2 and save it to the motor
        SparkMaxConfig stage2Config = new SparkMaxConfig();
        stage2Config.idleMode(IdleMode.kBrake); // Brake so the stage doesn't fall
        stage2Config.inverted(false);
        stage2Config.absoluteEncoder.inverted(false);

        // Configure the encoder so they automaticlly convert the motors rotation to meters
        double stage2Conversion = (Constants.ElevatorConstants.Stage2.DRUM_RADIUS.in(Meter) * 2 * Math.PI)
                / Constants.ElevatorConstants.Stage2.GEAR_RATIO;
        stage2Config.encoder.positionConversionFactor(stage2Conversion);
        stage2Config.encoder.velocityConversionFactor(stage2Conversion);
        stage2Motor.configure(stage2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Configure PIDS
         */

        // Define the trapizoid profile for the pids
        TrapezoidProfile.Constraints stage1Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage1.MAX_VELOCITY, Constants.ElevatorConstants.Stage1.MAX_ACCELERATION);
        TrapezoidProfile.Constraints stage2Constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.Stage2.MAX_VELOCITY, Constants.ElevatorConstants.Stage2.MAX_ACCELERATION);

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

        // Configure feed foward
        stage1FeedFoward = new ElevatorFeedforward(
                Constants.ElevatorConstants.Stage1.S.in(Volt),
                Constants.ElevatorConstants.Stage1.G.in(Volt),
                Constants.ElevatorConstants.Stage1.V.in(Volt),
                Constants.ElevatorConstants.Stage1.A.in(Volt));
        stage2FeedFoward = new ElevatorFeedforward(
                Constants.ElevatorConstants.Stage2.S.in(Volt),
                Constants.ElevatorConstants.Stage2.G.in(Volt),
                Constants.ElevatorConstants.Stage2.V.in(Volt),
                Constants.ElevatorConstants.Stage2.A.in(Volt));

        /*
         * Initalize simulation
         */
        if (Robot.isSimulation()) {
            System.out.println("Creating elevator simulations");
            stage1ElevatorSim = new ElevatorSim(
                    stage1Gearbox,
                    Constants.ElevatorConstants.Stage1.GEAR_RATIO,
                    Constants.ElevatorConstants.Stage1.MASS.in(Kilogram),
                    Constants.ElevatorConstants.Stage1.DRUM_RADIUS.in(Meter),
                    0,
                    this.stage1MaxHeight,
                    true,
                    0,
                    0.02,
                    0);

            stage2ElevatorSim = new ElevatorSim(
                    stage2Gearbox,
                    Constants.ElevatorConstants.Stage2.GEAR_RATIO,
                    Constants.ElevatorConstants.Stage2.MASS.in(Kilogram),
                    Constants.ElevatorConstants.Stage2.DRUM_RADIUS.in(Meter),
                    0,
                    this.stage2MaxHeight,
                    true,
                    0,
                    0.02,
                    0);
        }

        System.out.println("Created ElevatorSubsystem");
    }

    public Distance getStage1Setpoint() {
        return Meter.of(stage1Controller.getGoal().position);
    }

    public Distance getStage2Setpoint() {
        return Meter.of(stage2Controller.getGoal().position);
    }

    public void setStage1Setpoint(Distance height) {
        stage1Controller.setGoal(
                MathUtil.clamp(height.in(Meter), 0, this.stage1MaxHeight));
    }

    public void setStage2Setpoint(Distance height) {
        stage2Controller.setGoal(
                MathUtil.clamp(height.in(Meter), 0, this.stage2MaxHeight));
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
        return Meter.of(stage2Encoder.getPosition());
    }

    public LinearVelocity getStage1HeightVelocity() {
        return MetersPerSecond.of(stage1Encoder.getVelocity() / 60);
    }

    public LinearVelocity getStage2HeightVelocity() {
        return MetersPerSecond.of(stage2Encoder.getVelocity() / 60);
    }

    /**
     * Sets the setpoint for stage 1 to the stages current height
     */
    public void resetStage1Setpoint() {
        setStage1Setpoint(getStage1Height());
    }

    /**
     * Sets the setpoint for stage 2 to the stages current height
     */
    public void resetStage2Setpoint() {
        setStage2Setpoint(getStage2Height());
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
    public Distance getCarpetElevatorHeight() {
        return getStage1Height().plus(getStage2Height()).plus(getPivotPointOffset(true));
    }

    /**
     * Checks if a target height is reachable by the elevator
     * 
     * @param height The target height from the ground in meters
     * @return Whether the target height is posible
     */
    public boolean checkGlobalHeightPossible(Distance height) {

        // Because the elevator even at its lowest is a little bit off the ground its base height has to be subtracted from the overall height to get the local height
        double elevatorLocalHeight = (height.minus(getPivotPointOffset(true))).in(Meter);

        return elevatorLocalHeight >= 0 && elevatorLocalHeight <= this.maxHeight;
    }

    /**
     * Sets the overall height of the elevator by moving both stages
     * 
     * @param targetHeight The target height of the elevator relative to the ground in meters
     */
    public void setOverallHeight(Distance targetHeight) {

        // Because the target height is based on the distance from the carpet it has to be converted to elevator height
        double targetHeightMeters = targetHeight.in(Meter) - getPivotPointOffset(true).in(Meter);

        // Clamp the target height so it is within the possible range of the elevator
        targetHeightMeters = MathUtil.clamp(targetHeightMeters, 0, this.maxHeight);

        // Get the percent of the total height the elevator is going to
        double targetRatio = targetHeightMeters / maxHeight;

        double target1Height = MathUtil.clamp(
                this.stage1MaxHeight * targetRatio, 0,
                this.stage1MaxHeight);
        double target2Height = MathUtil.clamp(
                this.stage2MaxHeight * targetRatio, 0,
                this.stage2MaxHeight);

        // Set each elevators setpoint to the calculated heights in meters
        setStage1Setpoint(Meter.of(target1Height));
        setStage2Setpoint(Meter.of(target2Height));
    }

    @Override
    public void simulationPeriodic() {
        stage1ElevatorSim.setInput(stage1MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        stage2ElevatorSim.setInput(stage2MotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update every 20 miliseconds
        stage1ElevatorSim.update(0.02);
        stage2ElevatorSim.update(0.02);

        // Iterate the simulation of both motors
        stage1MotorSim.iterate(stage1ElevatorSim.getVelocityMetersPerSecond() * 60,
                RoboRioSim.getVInVoltage(), 0.02);

        stage2MotorSim.iterate(stage2ElevatorSim.getVelocityMetersPerSecond() * 60,
                RoboRioSim.getVInVoltage(), 0.02);

        // Add the current voltage as a load to the battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                stage1ElevatorSim.getCurrentDrawAmps() + stage2ElevatorSim.getCurrentDrawAmps()));

    }

    @Override
    public void periodic() {

        double stage1VoltsOutput = MathUtil.clamp(
                stage1Controller.calculate(getStage1Height().in(Meter)) + stage1FeedFoward
                        .calculateWithVelocities(getStage1HeightVelocity().in(MetersPerSecond),
                                stage1Controller.getGoal().velocity),
                -10, 10);
        stage1Motor.setVoltage(stage1VoltsOutput);

        double stage2VoltsOutput = MathUtil.clamp(
                stage2Controller.calculate(getStage2Height().in(Meter)) + stage2FeedFoward
                        .calculateWithVelocities(getStage2HeightVelocity().in(MetersPerSecond),
                                stage2Controller.getGoal().velocity),
                -10, 10);
        stage2Motor.setVoltage(stage2VoltsOutput);
    }
}
