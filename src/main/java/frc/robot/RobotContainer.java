// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utilities.controls.CustomCommandXboxController;

public class RobotContainer {

  // Initalize driver controller and stream
  public static final CustomCommandXboxController driverController = new CustomCommandXboxController(
      Constants.DriverConstants.PORT, Constants.DriverConstants.DEADBAND,
      Constants.DriverConstants.LEFT_JOYSTICK_EXPONENT,
      Constants.DriverConstants.RIGHT_JOYSTICK_EXPONENT, Constants.DriverConstants.TRIGGER_EXPONENT);

  public RobotContainer() {
    System.out.println("Configuring robot container");

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
