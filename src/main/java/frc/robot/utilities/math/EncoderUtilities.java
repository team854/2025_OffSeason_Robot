package frc.robot.utilities.math;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public final class EncoderUtilities {
    public static Angle clampAbsoluteEncoder(Angle angle) {
        double angleValue = angle.in(Degree) % 360.0;

        if (angleValue < 0) {
            angleValue = 360 + angleValue;
        }
        
        return Degree.of(angleValue);
    }

    /**
     * Calculates the zero offset that should be provided to the encoder in degrees (Bettween 0 - 360);
     * 
     * @param currentAngle The current angle that the encoder thinks its at
     * @param knownAngle The angle that the encoder is actually at
     * @param currentZeroOffset The current zero offset that is being applied
     * @return The zero offset that should be applied to the encoder
     */
    public static Angle calculateZeroOffset(Angle currentAngle, Angle knownAngle, Angle currentZeroOffset) {
        return clampAbsoluteEncoder(knownAngle.minus(currentAngle).plus(currentZeroOffset));
    }
}
