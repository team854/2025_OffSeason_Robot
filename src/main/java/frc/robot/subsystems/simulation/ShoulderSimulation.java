package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderSimulation extends SingleJointedArmSim {
    private boolean simulateGravity;
    private double minAngle;
    private double maxAngle;
    private double armLenMeters;

    private double pivotVerticalAcceleration = 0;

    /**
     * Creates a simulated arm mechanism.https://www.youtube.com/watch?v=h3TD3qKDO0U
     *
     * @param gearbox The type of and number of motors in the arm gearbox.
     * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
     * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
     * @param armLengthMeters The length of the arm.
     * @param minAngleRads The minimum angle that the arm is capable of.
     * @param maxAngleRads The maximum angle that the arm is capable of.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingAngleRads The initial position of the Arm simulation in radians.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    public ShoulderSimulation(
            DCMotor gearbox,
            double gearing,
            double jKgMetersSquared,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double startingAngleRads,
            double... measurementStdDevs) {
        super(gearbox, gearing, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity,
                startingAngleRads,
                measurementStdDevs);

        this.simulateGravity = simulateGravity;
        this.minAngle = minAngleRads;
        this.maxAngle = maxAngleRads;
        this.armLenMeters = armLengthMeters;
    }

    public void updatePivotVerticalAcceleration(LinearAcceleration verticalAcceleration) {
        this.pivotVerticalAcceleration = verticalAcceleration.in(MetersPerSecondPerSecond);
    }

    /**
    * Updates the state of the arm.
    *
    * @param currentXhat The current state estimate.
    * @param u The system inputs (voltage).
    * @param dtSeconds The time difference between controller updates.
    */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
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
                    Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                    if (simulateGravity) {
                        double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(x.get(0, 0)) / armLenMeters;
                        xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                    }

                    double alphaForce = 3.0 / 2.0 * -this.pivotVerticalAcceleration * Math.cos(x.get(0, 0)) / armLenMeters;
                    xdot = xdot.plus(VecBuilder.fill(0, alphaForce));
                    
                    return xdot;
                },
                currentXhat,
                u,
                dtSeconds);

        // We check for collision after updating xhat
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minAngle, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxAngle, 0);
        }
        return updatedXhat;
    }
}
