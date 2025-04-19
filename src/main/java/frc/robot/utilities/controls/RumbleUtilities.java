package frc.robot.utilities.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public final class RumbleUtilities {
    public static void rumbleCommandFailed() {
        new SequentialCommandGroup(RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kRightRumble, 0.8, 0.1),
                                    new WaitCommand(0.08),
                                    RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kRightRumble, 0.8, 0.1),
                                    new WaitCommand(0.08),
                                    RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kRightRumble, 0.8, 0.1)).schedule();
    }

    public static void rumbleCommandFullControlTaken() {
        new SequentialCommandGroup(RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.9, 0.15),
                                    new WaitCommand(0.1),
                                    RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kLeftRumble, 1, 0.15)).schedule();
    }

    public static void rumbleCommandFullControlGiven() {
        new SequentialCommandGroup(RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kLeftRumble, 1, 0.15),
                                    new WaitCommand(0.1),
                                    RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.9, 0.15)).schedule();
    }
}
