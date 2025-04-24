package frc.robot.utilities.controls;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Heavily inspired by https://github.com/frc4451/Riptide2025/blob/main/src/main/java/frc/robot/util/CommandCustomXboxController.java
public class CustomCommandXboxController extends CommandXboxController {
    private final int port;
    private final double deadband;
    private final double leftExponent;
    private final double rightExponent;
    private final double triggerExponent;

    public CustomCommandXboxController(int port, double deadband, double leftExponent, double rightExponent, double triggerExponent) {
        super(port);

        this.port = port;
        this.deadband = deadband;
        this.leftExponent = leftExponent;
        this.rightExponent = rightExponent;
        this.triggerExponent = triggerExponent;
    }

    @Override
    public double getLeftX() {
        return applyExponent(applyJoystickDeadband(super.getLeftX()), leftExponent);
    }

    @Override
    public double getLeftY() {
        return applyExponent(applyJoystickDeadband(super.getLeftY()), leftExponent);
    }

    @Override
    public double getRightX() {
        return applyExponent(applyJoystickDeadband(super.getRightX()), rightExponent);
    }

    @Override
    public double getRightY() {
        return applyExponent(applyJoystickDeadband(super.getRightY()), rightExponent);
    }

    @Override
    public double getLeftTriggerAxis() {
        return applyExponent(applyJoystickDeadband(super.getLeftTriggerAxis()), triggerExponent);
    }

    @Override
    public double getRightTriggerAxis() {
        return applyExponent(applyJoystickDeadband(super.getRightTriggerAxis()), triggerExponent);
    }

    public Trigger povUpDirection() {
        return new Trigger(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return pov(0).getAsBoolean() || pov(45).getAsBoolean() || pov(315).getAsBoolean();
            }
        });
    }

    public Trigger povDownDirection() {
        return new Trigger(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return pov(180).getAsBoolean() || pov(225).getAsBoolean() || pov(135).getAsBoolean();
            }
        });
    }

    public Trigger povRightDirection() {
        return new Trigger(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return pov(90).getAsBoolean() || pov(45).getAsBoolean() || pov(135).getAsBoolean();
            }
        });
    }

    public Trigger povLeftDirection() {
        return new Trigger(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return pov(270).getAsBoolean() || pov(315).getAsBoolean() || pov(225).getAsBoolean();
            }
        });
    }

    public boolean noDirectionsHeld() {
        return (getLeftX() == 0 && getLeftY() == 0 && getRightX() == 0 && getRightY() == 0);
    }

    public Command setRumbleCommand(RumbleType type, double strength) {
        return Commands.startEnd(() -> super.setRumble(type, strength), () -> super.setRumble(type, 0));
    }

    public Command setRumbleSecondsCommand(RumbleType type, double strength, double seconds) {
        return setRumbleCommand(type, strength).withTimeout(seconds);
    }

    public Command setRumbleBlinkCommand(RumbleType type, double strength, double secondsOn, double secondsOff, int loops) {
        return Commands.repeatingSequence(setRumbleSecondsCommand(type, strength, secondsOn), Commands.waitSeconds(secondsOff)).withTimeout((secondsOn + secondsOff) * loops);
    }

    public Trigger multiButtonTrigger(int... buttonIDs) {
        List<JoystickButton> joystickButtons = new ArrayList<>();
        for (int buttonID : buttonIDs) {
            joystickButtons.add(new JoystickButton(super.getHID(), buttonID));
        }

        return new MultiButtonTrigger(joystickButtons.toArray(new JoystickButton[0]));
    }

    private double applyExponent(double value, double exponent) {
        return Math.pow(Math.abs(value), exponent) * Math.signum(value);
    }

    private double applyJoystickDeadband(double value) {
        return MathUtil.applyDeadband(value, this.deadband);
    }
}
