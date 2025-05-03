package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorSimulation {
    /** The plant that represents the linear system. */
    protected LinearSystem<N2, N1, N2> systemMatrix;

    /** State vector. */
    protected Matrix<N2, N1> stateVector;

    /** Input vector. */
    protected Matrix<N1, N1> inputVector;

    /** Output vector. */
    protected Matrix<N2, N1> outputVector;

    /** The standard deviations of measurements, used for adding noise to the measurements. */
    protected final Matrix<N2, N1> measurementStdDevs;

    /** Gearbox for the elevator. */
    private final DCMotor gearbox;

    /** The gearing between the motors and the output. */
    private final double gearRatio;

    /** The radius of the drum. */
    private final double drumRadius;

    /** The min allowable height for the elevator. */
    private final double minHeight;

    /** The max allowable height for the elevator. */
    private final double maxHeight;

    /** Whether the simulator should simulate gravity. */
    private final boolean simulateGravity;

    /** The mass of the elevator. */
    private double mass;

    public ElevatorSimulation(
            DCMotor gearbox,
            double gearRatio,
            Mass mass,
            Distance drumRadius,
            Distance minHeight,
            Distance maxHeight,
            boolean simulateGravity,
            Distance startingHeight,
            double... measurementStdDevs) {

        this.gearbox = gearbox;
        this.gearRatio = gearRatio;
        this.drumRadius = drumRadius.in(Meter);
        this.minHeight = minHeight.in(Meter);
        this.maxHeight = maxHeight.in(Meter);
        this.simulateGravity = simulateGravity;

        this.mass = mass.in(Kilogram);
        this.systemMatrix = LinearSystemId.createElevatorSystem(this.gearbox, this.mass, this.drumRadius,
                this.gearRatio);

        this.stateVector = new Matrix<>(new SimpleMatrix(this.systemMatrix.getA().getNumRows(), 1));
        this.inputVector = new Matrix<>(new SimpleMatrix(this.systemMatrix.getB().getNumCols(), 1));
        this.outputVector = new Matrix<>(new SimpleMatrix(this.systemMatrix.getC().getNumRows(), 1));

        if (measurementStdDevs.length == 0) {
            this.measurementStdDevs = new Matrix<>(new SimpleMatrix(this.systemMatrix.getC().getNumRows(), 1));
        } else {
            if (measurementStdDevs.length != this.systemMatrix.getC().getNumRows()) {
                throw new MatrixDimensionException(
                        "Malformed measurementStdDevs! Got "
                                + measurementStdDevs.length
                                + " elements instead of "
                                + this.systemMatrix.getC().getNumRows());
            }

            this.measurementStdDevs = new Matrix<>(new SimpleMatrix(measurementStdDevs));
        }

        setState(startingHeight, MetersPerSecond.of(0.0));
    }

    public void setWeight(Mass weight) {
        // Check to make sure the weight is different from the current mass
        if (weight.in(Kilogram) == this.mass) {
            return;
        }

        this.mass = weight.in(Kilogram);

        this.systemMatrix = LinearSystemId.createElevatorSystem(this.gearbox, this.mass, this.drumRadius, this.gearRatio);
        this.outputVector = this.systemMatrix.calculateY(this.stateVector, this.inputVector);
    }

    /**
    * Updates the simulation.
    *
    * @param dtSeconds The time between updates.
    */
    public void update(double dtSeconds) {
        // Update x. By default, this is the linear system dynamics xₖ₊₁ = Axₖ + Buₖ.
        this.stateVector = updateX(this.stateVector, this.inputVector, dtSeconds);

        // yₖ = Cxₖ + Duₖ
        this.outputVector = this.systemMatrix.calculateY(this.stateVector, this.inputVector);

        // Add measurement noise.
        if (measurementStdDevs != null) {
            this.outputVector = this.outputVector.plus(StateSpaceUtil.makeWhiteNoiseVector(this.measurementStdDevs));
        }
    }

    /**
    * Returns whether the elevator would hit the lower limit.
    *
    * @param height The elevator height.
    * @return Whether the elevator would hit the lower limit.
    */
    public boolean wouldHitLowerLimit(Distance height) {
        return height.in(Meter) <= this.minHeight;
    }

    /**
    * Returns whether the elevator would hit the upper limit.
    *
    * @param height The elevator height.
    * @return Whether the elevator would hit the upper limit.
    */
    public boolean wouldHitUpperLimit(Distance height) {
        return height.in(Meter) >= this.maxHeight;
    }

    /**
    * Sets the elevator's state. The new position will be limited between the minimum and maximum
    * allowed heights.
    *
    * @param height The new position.
    * @param linearVelocity The new linear velocity.
    */
    public final void setState(Distance height, LinearVelocity linearVelocity) {
        this.stateVector = VecBuilder.fill(MathUtil.clamp(height.in(Meter), this.minHeight, this.maxHeight),
                linearVelocity.in(MetersPerSecond));

        // Update the output to reflect the new state.
        //
        //   yₖ = Cxₖ + Duₖ
        this.outputVector = this.systemMatrix.calculateY(this.stateVector, this.inputVector);
    }

    /**
    * Returns the position of the elevator.
    *
    * @return The position of the elevator.
    */
    public Distance getPosition() {
        return Meter.of(this.outputVector.get(0, 0));
    }

    /**
    * Returns the velocity of the elevator.
    *
    * @return The velocity of the elevator.
    */
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(this.outputVector.get(1, 0));
    }

    /**
    * Returns the elevator current draw.
    *
    * @return The elevator current draw.
    */
    public Current getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        // v = r w, so w = v/r
        double kA = 1 / this.systemMatrix.getB().get(1, 0);
        double kV = -this.systemMatrix.getA().get(1, 1) * kA;
        double motorVelocityRadPerSec = this.stateVector.get(1, 0) * kV * this.gearbox.KvRadPerSecPerVolt;
        var appliedVoltage = this.inputVector.get(0, 0);
        return Amp.of(this.gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
                * Math.signum(appliedVoltage));
    }

    /**
    * Sets the input voltage for the arm.
    *
    * @param volts The input voltage.
    */
    public void setInputVoltage(Voltage volts) {
        this.inputVector = new Matrix<>(new SimpleMatrix(this.inputVector.getNumRows(), 1, true, volts.in(Volt)));
        clampInput(RobotController.getBatteryVoltage());
    }

    /**
    * Clamp the input vector such that no element exceeds the maximum allowed value. If any does, the
    * relative magnitudes of the input will be maintained.
    *
    * @param maxInput The maximum magnitude of the input vector after clamping.
    */
    protected void clampInput(double maxInput) {
        this.inputVector = StateSpaceUtil.desaturateInputVector(this.inputVector, maxInput);
    }

    /**
    * Updates the state of the elevator.
    *
    * @param currentXhat The current state estimate.
    * @param input The system inputs (voltage).
    * @param dtSeconds The time difference between controller updates.
    */
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> input, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat = NumericalIntegration.rkdp(
                (x, _u) -> {
                    Matrix<N2, N1> xdot = this.systemMatrix.getA().times(x).plus(this.systemMatrix.getB().times(_u));
                    if (this.simulateGravity) {
                        xdot = xdot.plus(VecBuilder.fill(0, -9.8));
                    }
                    return xdot;
                },
                currentXhat,
                input,
                dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(Meter.of(updatedXhat.get(0, 0)))) {
            return VecBuilder.fill(minHeight, 0);
        }
        if (wouldHitUpperLimit(Meter.of(updatedXhat.get(0, 0)))) {
            return VecBuilder.fill(maxHeight, 0);
        }
        return updatedXhat;
    }
}
