package frc.robot.utilities.math;

import edu.wpi.first.math.geometry.Translation3d;

public final class VectorUtilities {
    /**
     * Calculates the dot product of two {@link Translation3d}
     * @param a The first {@link Translation3d}
     * @param b The second {@link Translation3d}
     * @return The dot product
     */
    public static double dotProduct(Translation3d a, Translation3d b) {
        return (a.getX() * b.getX()) + (a.getY() * b.getY()) + (a.getZ() * b.getZ());
    }

    /**
     * Calculates the cross product of two {@link Translation3d}
     * @param a The first {@link Translation3d}
     * @param b The second {@link Translation3d}
     * @return The cross product
     */
    public static Translation3d crossProduct(Translation3d a, Translation3d b) {
        return new Translation3d(
            (a.getY() * b.getZ()) - (a.getZ() * b.getY()),
            (a.getZ() * b.getX()) - (a.getX() * b.getZ()),
            (a.getX() * b.getY()) - (a.getY() * b.getX())
        );
    }
}
