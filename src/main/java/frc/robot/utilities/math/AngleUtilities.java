package frc.robot.utilities.math;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;

public final class AngleUtilities {
    public static Angle normalizeAngle(Angle angle) {
        double degrees = angle.in(Degree) % 360;
        
        if (degrees > 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }

        return Degree.of(degrees);
    }

    public static Angle getPerpendicularAngle(Angle angle) {
        return normalizeAngle(angle.minus(Degree.of(90)));
    }
}
