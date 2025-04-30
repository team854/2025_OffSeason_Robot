package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
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
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ShoulderSimulation {
    /** The plant that represents the linear system. */
    protected LinearSystem<N2, N1, N2> system;

    /** State vector. */
    protected Matrix<N2, N1> stateVector;

    /** Input vector. */
    protected Matrix<N1, N1> inputVector;

    /** Output vector. */
    protected Matrix<N2, N1> outputVector;

    /** The standard deviations of measurements, used for adding noise to the measurements. */
    protected final Matrix<N2, N1> measurementStdDevs;

    // The gearbox for the arm.
    private final DCMotor gearbox;

    // The gearing between the motors and the output.
    private final double gearRatio;

    // The mass of the arm.
    private double mass;

    // The length of the arm.
    private final double armLength;

    // The minimum angle that the arm is capable of.
    private final double minAngle;

    // The maximum angle that the arm is capable of.
    private final double maxAngle;

    // Whether the simulator should simulate gravity.
    private final boolean simulateGravity;

    private double pivotVerticalAcceleration = 0;
    private double pivotForwardAcceleration = 0;

    public ShoulderSimulation(
        DCMotor gearbox,
        double gearing,
        Mass mass,
        Distance armLength,
        Angle minAngle,
        Angle maxAngle,
        boolean simulateGravity,
        Angle startingAngle,
        double... measurementStdDevs) {
        
        this.gearbox = gearbox;
        this.gearRatio = gearing;
        this.armLength = armLength.in(Meter);
        this.minAngle = minAngle.in(Radian);
        this.maxAngle = maxAngle.in(Radian);
        this.simulateGravity = simulateGravity;

        this.system = LinearSystemId.createSingleJointedArmSystem(this.gearbox, estimateMOI(armLength, mass), this.gearRatio);
        this.mass = mass.in(Kilogram);

        this.stateVector = new Matrix<>(new SimpleMatrix(this.system.getA().getNumRows(), 1));
        this.inputVector = new Matrix<>(new SimpleMatrix(this.system.getB().getNumCols(), 1));
        this.outputVector = new Matrix<>(new SimpleMatrix(this.system.getC().getNumRows(), 1));

        if (measurementStdDevs.length == 0) {
            this.measurementStdDevs = new Matrix<>(new SimpleMatrix(this.system.getC().getNumRows(), 1));
        } else {
            if (measurementStdDevs.length != this.system.getC().getNumRows()) {
                throw new MatrixDimensionException(
                    "Malformed measurementStdDevs! Got "
                        + measurementStdDevs.length
                        + " elements instead of "
                        + this.system.getC().getNumRows());
            }

            this.measurementStdDevs = new Matrix<>(new SimpleMatrix(measurementStdDevs));
        }

        setState(startingAngle, RadiansPerSecond.of(0.0));
    }

    public void setWeight(Mass weight) {
        // Check to make sure the weight is different from the current mass
        if (weight.in(Kilogram) == this.mass) {
            return;
        }

        System.out.println("Updating weight to " + weight.in(Kilogram) + "kg");

        this.system = LinearSystemId.createSingleJointedArmSystem(this.gearbox, estimateMOI(Meter.of(this.armLength), weight), this.gearRatio);
        this.outputVector = this.system.calculateY(this.stateVector, this.inputVector);

        this.mass = weight.in(Kilogram);
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
        this.outputVector = this.system.calculateY(this.stateVector, this.inputVector);

        // Add measurement noise.
        if (measurementStdDevs != null) {
            this.outputVector = this.outputVector.plus(StateSpaceUtil.makeWhiteNoiseVector(this.measurementStdDevs));
        }
    }

    /**
    * Returns whether the arm would hit the lower limit.
    *
    * @param angle The current arm angle.
    * @return Whether the arm would hit the lower limit.
    */
    public boolean wouldHitLowerLimit(Angle angle) {
        return angle.in(Radian) <= this.minAngle;
    }

    /**
    * Returns whether the arm would hit the upper limit.
    *
    * @param angle The current arm angle.
    * @return Whether the arm would hit the upper limit.
    */
    public boolean wouldHitUpperLimit(Angle angle) {
        return angle.in(Radian) >= this.maxAngle;
    }

    /**
    * Returns the current arm angle.
    *
    * @return The current arm angle.
    */
    public Angle getAngle() {
        return Radian.of(this.outputVector.get(0, 0));
    }

    /**
    * Returns the current arm velocity.
    *
    * @return The current arm velocity.
    */
    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(this.outputVector.get(1, 0));
    }

    /**
    * Returns the arm current draw.
    *
    * @return The arm current draw.
    */
    public Current getCurrentDrawAmps() {
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        var motorVelocity = this.stateVector.get(1, 0) * gearRatio;
        return Amp.of(gearbox.getCurrent(motorVelocity, this.inputVector.get(0, 0)) * Math.signum(this.inputVector.get(0, 0)));
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
    * Calculates a rough estimate of the moment of inertia of an arm given its length and mass.
    *
    * @param lengthMeters The length of the arm.
    * @param massKg The mass of the arm.
    * @return The calculated moment of inertia.
    */
    public static double estimateMOI(Distance length, Mass mass) {
        return 1.0 / 3.0 * mass.in(Kilogram) * length.in(Meter) * length.in(Meter);
    }

    /**
    * Sets the arm's state. The new angle will be limited between the minimum and maximum allowed
    * limits.
    *
    * @param angle The new angle.
    * @param angularVelocity The new angular velocity.
    */
    public final void setState(Angle angle, AngularVelocity angularVelocity) {
        this.stateVector = VecBuilder.fill(MathUtil.clamp(angle.in(Radian), this.minAngle, this.maxAngle), angularVelocity.in(RadiansPerSecond));

        // Update the output to reflect the new state.
        //
        //   yₖ = Cxₖ + Duₖ
        this.outputVector = this.system.calculateY(this.stateVector, this.inputVector);

    }

    public void updatePivotVerticalAcceleration(LinearAcceleration verticalAcceleration) {
        this.pivotVerticalAcceleration = verticalAcceleration.in(MetersPerSecondPerSecond);
    }

    public void updatePivotForwardAcceleration(LinearAcceleration forwardAcceleration) {
        this.pivotForwardAcceleration = forwardAcceleration.in(MetersPerSecondPerSecond);
    }

    /**
    * Updates the state of the arm.
    *
    * @param currentXhat The current state estimate.
    * @param input The system inputs (voltage).
    * @param dtSeconds The time difference between controller updates.
    */
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> input, double dtSeconds) {
        // The torque on the arm is given by τ = F⋅r, where F is the force applied by
        // gravity and r the distance from pivot to center of mass. Recall from
        // dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
        // torque on the arm, J is the mass-moment of inertia about the pivot axis,
        // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
        //
        // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
        //
        //   α = (m⋅g⋅cos(θ))⋅r/J
        //
        // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
        // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
        // rod rotating about it's end, where L is the overall rod length. The mass
        // distribution is assumed to be uniform. Substitute r=L/2 to find:
        //
        //   α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
        //   α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
        //   α = 3/2⋅g⋅cos(θ)/L
        //
        // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
        //
        //   f(x, u) = Ax + Bu + [0  α]ᵀ
        //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ

        Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                    Matrix<N2, N1> xdot = this.system.getA().times(x).plus(this.system.getB().times(_u));
                    if (this.simulateGravity) {
                        double alphaGrav = (3.0 / 2.0) * -9.8 * Math.cos(x.get(0, 0)) / this.armLength;
                        xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                    }

                    // Handle verticle movement from the elevator
                    double vertForce = (6.0 / 5.0) * -this.pivotVerticalAcceleration * Math.cos(x.get(0, 0)) / this.armLength;
                    xdot = xdot.plus(VecBuilder.fill(0, vertForce));
                    
                    // Handle forward movement from the swerve base
                    double fwdForce = (6.0 / 5.0) * this.pivotForwardAcceleration * Math.sin(x.get(0, 0)) / this.armLength;
                    xdot = xdot.plus(VecBuilder.fill(0, fwdForce));

                    return xdot;
                },
                currentXhat,
                input,
                dtSeconds);

        // We check for collision after updating xhat
        if (wouldHitLowerLimit(Radian.of(updatedXhat.get(0, 0)))) {
            return VecBuilder.fill(this.minAngle, 0);
        }
        if (wouldHitUpperLimit(Radian.of(updatedXhat.get(0, 0)))) {
            return VecBuilder.fill(this.maxAngle, 0);
        }
        return updatedXhat;
    }
}
