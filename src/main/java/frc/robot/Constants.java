package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

public final class Constants {
	public static class Starting {
		public static final Distance X = Feet.of(24.95316667);
		public static final Distance Y = Meter.of(APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2);
	}

	public static final boolean USE_WELDED_FIELD = false;
	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(USE_WELDED_FIELD ? AprilTagFields.k2025ReefscapeWelded : AprilTagFields.k2025ReefscapeAndyMark);

	public static class SwerveConstants {

		public static class SwerveModuleConstants {
			public static class BackLeft {
				public static class Location {
					public static final Distance FRONT = Inch.of(-14.75); // Inches
					public static final Distance LEFT = Inch.of(9.875); // Inches
				}

				public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(308.8381620); // Degrees

				public static class DriveMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 23;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}

				public static class AngleMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 33;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = true;
				}

				public static class Encoder {
					public static final String TYPE = "thrifty";
					public static final int ID = 3;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}
			}

			public static class BackRight {
				public static class Location {
					public static final Distance FRONT = Inch.of(-14.75); // Inches
					public static final Distance LEFT = Inch.of(-9.875); // Inches
				}

				public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(150.9974208105); // Degrees

				public static class DriveMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 22;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}

				public static class AngleMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 32;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = true;
				}

				public static class Encoder {
					public static final String TYPE = "thrifty";
					public static final int ID = 2;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}
			}

			public static class FrontLeft {
				public static class Location {
					public static final Distance FRONT = Inch.of(14.75); // Inches
					public static final Distance LEFT = Inch.of(9.875); // Inches
				}

				public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(19.47493551); // Degrees

				public static class DriveMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 20;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}

				public static class AngleMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 30;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = true;
				}

				public static class Encoder {
					public static final String TYPE = "thrifty";
					public static final int ID = 0;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}
			}

			public static class FrontRight {
				public static class Location {
					public static final Distance FRONT = Inch.of(14.75); // Inches
					public static final Distance LEFT = Inch.of(-9.875); // Inches
				}

				public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(112.15935540); // Degrees

				public static class DriveMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 21;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}

				public static class AngleMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 31;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = true;
				}

				public static class Encoder {
					public static final String TYPE = "thrifty";
					public static final int ID = 1;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = false;
				}
			}
		}

		public static class Imu {
			public static final String TYPE = "navx";
			public static final int ID = 0;
			public static final Object CANBUS = null;
			public static final boolean INVERTED = false;
			public static final Angle OFFSET = Degree.of(0.0); // Degrees
			public static final double ANGULAR_VELOCITY_COEFF = 0.1;
		}

		public static class MotorConstants {
			public static class Angle {
				public static final Time RAMP_RATE = Second.of(0); // Seconds
				public static final double GEAR_RATIO = 21.4285714286;
				public static final double FACTOR = 0;
				public static final Current CURRENT_LIMIT = Amp.of(20.0); // Amps

				public static class Pidf {
					public static final double P = 0.055;
					public static final double I = 0;
					public static final double D = 0.02;
					public static final double F = 0;
					public static final double IZ = 0;
				}
			}

			public static class Drive {
				public static final Time RAMP_RATE = Second.of(0); // Seconds
				public static final double GEAR_RATIO = 6.75;
				public static final double FACTOR = 0;
				public static final Current CURRENT_LIMIT = Amp.of(40.0); // Amps

				public static class Pidf {
					public static final double P = 0.006;
					public static final double I = 0;
					public static final double D = 0;
					public static final double F = 0;
					public static final double IZ = 0;
				}
			}

			public static final Voltage OPTIMAL_VOLTAGE = Volt.of(12.0); // Volts
		}

		public static class TranslationPID {
			public static final double P = 7;
			public static final double I = 0;
			public static final double D = 0.1;
		}

		public static class HeadingPID {
			public static final double P = 7;
			public static final double I = 0;
			public static final double D = 0.04;
		}

		public static class WheelConstants {
			public static final double WHEEL_GRIP_COF = 1.19;
			public static final Distance DIAMETER = Inch.of(4.0); // Inches
		}

		public static final double ANGLE_JOYSTICK_RADIUS_DEADBAND = 0.5;
		public static final List<String> MODULE_FILES = List.of("frontleft.json", "frontright.json", "backleft.json", "backright.json");
		public static final String SWERVECONFIGDIR = "swerve";
	}

	public static class RobotKinematicConstants {
		public static final Mass MASS = Pound.of(115.0); // Pounds
		public static final Distance WIDTH = Feet.of(2.58333333); // Feet
		public static final Distance LENGTH = Feet.of(3.41666667); // Feet
		public static final Distance HEIGHT_OFF_GROUND = Feet.of(0.033); // Feet
		public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(24.0); // Feet/Second
		public static final LinearVelocity MAX_ACHIEVABLE_SPEED = FeetPerSecond.of(24.0); // Feet/Second
		public static final LinearVelocity MAX_ACCELERATION = FeetPerSecond.of(6.5); // Feet/Second
		public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(540); // Degrees/Second
		public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(720); // Degrees/Second/Second
		public static final double MOI = 6.883;
		public static final String DRIVE_MOTOR_TYPE = "NEO";

		public static class BumperOffset {
			public static final Distance X = Feet.of(0); // Feet
			public static final Distance Y = Feet.of(0); // Feet
		}
	}

	public static class ElevatorConstants {
		public static class Stage1 {
			public static final int ID = 3;
			public static final double MAX_VELOCITY = 120; // Not sure the unit
			public static final double MAX_ACCELERATION = 200; // Not sure the unit
			public static final double P = 67;
			public static final double I = 0;
			public static final double D = 1.15;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(1.36); // Volts
			public static final Voltage V = Volt.of(17.0); // Volts/(Meters/Second)
			public static final Voltage A = Volt.of(0.2); // Volts/(Meters/Second^2)
			public static final Mass MASS = Pound.of(50); // Pounds
			public static final Distance DRUM_RADIUS = Inch.of(0.98110236); // Inches
			public static final double GEAR_RATIO = 18.5714;
			public static final Distance HARD_MAX_HEIGHT = Feet.of(2.25); // Feet
			public static final Distance TOLLERANCE = Feet.of(0.2); // Feet
		}

		public static class Stage2 {
			public static final int ID = 2;
			public static final double MAX_VELOCITY = 120; // Not sure the unit
			public static final double MAX_ACCELERATION = 200; // Not sure the unit
			public static final double P = 66;
			public static final double I = 0;
			public static final double D = 1.15;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(0.65); // Volts
			public static final Voltage V = Volt.of(14.52); // Volts/(Meters/Second)
			public static final Voltage A = Volt.of(0.04); // Volts/(Meters/Second^2)
			public static final Mass MASS = Pound.of(35); // Pounds
			public static final Distance DRUM_RADIUS = Inch.of(0.98110236); // Inches
			public static final double GEAR_RATIO = 15.7143;
			public static final Distance HARD_MAX_HEIGHT = Feet.of(2.12); // Feet
			public static final Distance TOLLERANCE = Feet.of(0.2); // Feet
		}

		public static final Distance ZERO_HEIGHTS_ABOVE_BASE = Feet.of(0.520); // Feet
	}

	public static class ArmConstants {
		public static class Shoulder {
			public static final int ID = 4;
			public static final double MAX_VELOCITY = 120; // Not sure the unit
			public static final double MAX_ACCELERATION = 200; // Not sure the unit
			public static final double P = 0.6;
			public static final double I = 0;
			public static final double D = 0.08;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(3.45); // Volts
			public static final Voltage V = Volt.of(0.01); // Volts/(Degrees/Second)
			public static final Voltage A = Volt.of(0.02); // Volts/(Degrees/Second^2)
			public static final Mass MASS = Pound.of(10.0); // Pounds
			public static final Angle MIN_ANGLE = Degree.of(-80.0); // Degrees
			public static final Angle MAX_ANGLE = Degree.of(80.0); // Degrees
			public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(0); // Degrees
			public static final double GEAR_RATIO = 50;
			public static final double ABSOLUTE_ENCODER_GEAR_RATIO = 1.85714;
			public static final Angle ABSOLUTE_ENCODER_PUSH_BACK = Degree.of(25.0); // Degrees
			public static final Distance CENTER_OFFSET_FOWARD = Feet.of(0.492126); // Feet
			public static final Distance STAGE_OFFSET_UP = Feet.of(0.958); // Feet
			public static final Angle TOLLERANCE = Degree.of(3.0); // Degrees
		}

		public static class Wrist {
			public static final int ID = 5;
			public static final double MAX_VELOCITY = 120; // Not sure the unit
			public static final double MAX_ACCELERATION = 200; // Not sure the unit
			public static final double P = 0.5;
			public static final double I = 0;
			public static final double D = 0.05;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(3.48); // Volts
			public static final Voltage V = Volt.of(0.01); // Volts/(Degrees/Second)
			public static final Voltage A = Volt.of(0.02); // Volts/(Degrees/Second^2)
			public static final Mass MASS = Pound.of(15.0); // Pounds
			public static final double MOI = 0.05; // Moment of innertia jKgMetersSquared
			public static final double GEAR_RATIO = 400;
			public static final Angle TOLLERANCE = Degree.of(2); // Degrees
		}

		public static class Intake {
			public static final int ID = 6;

			public static class Simulation {
				public static final Distance WIDTH = Feet.of(0.7); // Feet
				public static final Distance LENGTH = Feet.of(0.6); // Feet
			}
		}

		public static class IntakeSensor {
			public static final int ID = 0;
		}

		public static final Distance LENGTH = Feet.of(2.081208); // Feet

	}

	public static class DriverConstants {
		public static final int PORT = 0;
		public static final double DEADBAND = 0.1;
		public static final double TRANSLATION_SCALE = 1;
		public static final double ROTATION_SCALE = 0.7;
		public static final double LEFT_JOYSTICK_EXPONENT = 1.5;
		public static final double RIGHT_JOYSTICK_EXPONENT = 1.5;
		public static final double TRIGGER_EXPONENT = 1;
		public static final LinearVelocity CONTROL_ELEVATOR_SPEED = FeetPerSecond.of(1); // Feet/Second
		public static final AngularVelocity CONTROL_SHOULDER_SPEED = DegreesPerSecond.of(50); // Degrees/Second
		public static final AngularVelocity CONTROL_WRIST_SPEED = DegreesPerSecond.of(60); // Degrees/Second
		public static final double INTAKE_SPEED = -0.9; // Percent
		public static final double OUTTAKE_SPEED = 0.9; // Percent
		public static final double CLIMB_UP_SPEED = -1; // Percent
	}

	public static class DebugConstants {
		public static boolean DEBUG_VISION = true;
		public static boolean DEBUG_ELEVATOR = true;
		public static boolean DEBUG_ARM = true;
		public static boolean DEBUG_WRIST = true;
		public static boolean DEBUG_INTAKE = true;
		public static boolean DEBUG_SIMULATION = true;
		public static boolean DEBUG_PATHFINDING = true;
		public static boolean ANIMATE_ROBOT = true;
	}
}
