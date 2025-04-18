package frc.robot.utilities.controls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RumbleUtilities {
    public static void rumbleCommandFailed() {
        new SequentialCommandGroup().schedule(); // CREATE SOME RUMBLE PATTERNS 
    }
}
