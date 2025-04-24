package frc.robot.utilities.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Inspired by too https://github.com/frc-862/lightning/blob/develop/src/main/java/com/lightningrobotics/common/util/operator/trigger/TwoButtonTrigger.java
public class MultiButtonTrigger extends Trigger {

    public MultiButtonTrigger(BooleanSupplier... suppliers) {
        super(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                for (BooleanSupplier supplier : suppliers) {
                    if (supplier.getAsBoolean() == false) {
                        return false;
                    } 
                }

                return true;
            }
        });
    }
    
}
