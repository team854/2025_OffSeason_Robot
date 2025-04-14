package frc.robot.utilities.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Inspired by too https://github.com/frc-862/lightning/blob/develop/src/main/java/com/lightningrobotics/common/util/operator/trigger/TwoButtonTrigger.java
public class TwoButtonTrigger extends Trigger {
    public TwoButtonTrigger(JoystickButton b1, JoystickButton b2) {
        super(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return b1.getAsBoolean() && b2.getAsBoolean();
            }
        });
    }
    
}
