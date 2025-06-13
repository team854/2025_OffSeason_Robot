package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
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
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
					public static final boolean INVERTED = true;
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
					public static final boolean INVERTED = true;
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

				public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(223); // Degrees

				public static class DriveMotor {
					public static final String TYPE = "sparkmax_neo";
					public static final int ID = 20;
					public static final Object CANBUS = null;
					public static final boolean INVERTED = true;
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
					public static final boolean INVERTED = true;
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
				public static final Time RAMP_RATE = Second.of(0.15); // Seconds
				public static final double GEAR_RATIO = 21.4285714286;
				public static final double FACTOR = 0;
				public static final Current CURRENT_LIMIT = Amp.of(20.0); // Amps

				public static class Pidf {
					public static final double P = 0.01;
					public static final double I = 0;
					public static final double D = 0.1;
					public static final double F = 0;
					public static final double IZ = 0;
				}
			}

			public static class Drive {
				public static final Time RAMP_RATE = Second.of(0.15); // Seconds
				public static final double GEAR_RATIO = 6.75;
				public static final double FACTOR = 0;
				public static final Current CURRENT_LIMIT = Amp.of(40.0); // Amps

				public static class Pidf {
					public static final double P = 0.004;
					public static final double I = 0;
					public static final double D = 0;
					public static final double F = 0;
					public static final double IZ = 0;
				}
			}

			public static final Voltage OPTIMAL_VOLTAGE = Volt.of(12.0); // Volts
		}

		public static class TranslationPID {
			public static final double P = 6;
			public static final double I = 0;
			public static final double D = 0.15;
		}

		public static class HeadingPID {
			public static final double P = 5;
			public static final double I = 0;
			public static final double D = 0.04;
		}

		public static class WheelConstants {
			public static final double WHEEL_GRIP_COF = 1.19;
			public static final Distance DIAMETER = Inch.of(4.0); // Inches
		}

		public static final LinearVelocity TRANSLATION_ZERO_THRESHOLD = FeetPerSecond.of(0.05); // Feet/Second
		public static final AngularVelocity ROTATION_ZERO_THRESHOLD = DegreesPerSecond.of(0.05); // Degrees/Second
		public static final Distance TRANSLATION_ACCEPTABLE_ERROR = Feet.of(0.1); // Feet
		public static final Angle ROTATION_ACCEPTABLE_ERROR = Degree.of(2); // Degrees

		public static final boolean ENABLE_FEED_FORWARD = false; // Controls if feed forward should be enabled in the auto

		public static final double ANGLE_JOYSTICK_RADIUS_DEADBAND = 0.5;
		public static final List<String> MODULE_FILES = List.of("frontleft.json", "frontright.json", "backleft.json", "backright.json");
		public static final String SWERVECONFIGDIR = "swerve";
	}

	public static class RobotKinematicConstants {
		public static final Mass MASS = Pound.of(115.0); // Pounds
		public static final Distance WIDTH = Feet.of(2.58333333); // Feet
		public static final Distance LENGTH = Feet.of(3.42666667); // Feet
		public static final Distance HEIGHT_OFF_GROUND = Feet.of(0.033); // Feet
		public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(16.0); // Feet/Second
		public static final LinearVelocity MAX_ACHIEVABLE_SPEED = FeetPerSecond.of(14.5); // Feet/Second
		public static final LinearAcceleration MAX_ACCELERATION = FeetPerSecondPerSecond.of(6.5); // Feet/Second
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
		public static final boolean ENABLED = true;
		
		public static class Stage1 {
			public static final int ID = 3;
			public static final double MAX_VELOCITY = 220; // Not sure the unit
			public static final double MAX_ACCELERATION = 150; // Not sure the unit
			public static final double P = 100; // 75;
			public static final double I = 0;
			public static final double D = 0.5; // 1.1;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(0.2);//2.1); // Volts
			public static final Voltage V = Volt.of(0.1);//19.5); // Volts/(Meters/Second)
			public static final Voltage A = Volt.of(0.001);//0.2); // Volts/(Meters/Second^2)
			public static final Mass MASS = Pound.of(70); // Pounds (Including stage 2 and the arm)
			public static final Distance DRUM_RADIUS = Inch.of(1.106);//0.98110236); // Inches
			public static final double GEAR_RATIO = 45;
			public static final Distance HARD_MAX_HEIGHT = Feet.of(2.75); // Feet
			
		}

		public static class Stage2 {
			public static final int ID = 2;
			public static final double MAX_VELOCITY = 220; // Not sure the unit
			public static final double MAX_ACCELERATION = 150; // Not sure the unit
			public static final double P = 100;//75;
			public static final double I = 0;
			public static final double D = 0;//1.1;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(0.2);//1.2); // Volts
			public static final Voltage V = Volt.of(0.05);//17.5); // Volts/(Meters/Second)
			public static final Voltage A = Volt.of(0.001);//0.04); // Volts/(Meters/Second^2)
			public static final Mass MASS = Pound.of(45); // Pounds (Including the arm)
			public static final Distance DRUM_RADIUS = Inch.of(0.98110236); // Inches
			public static final double GEAR_RATIO = 34;//15.7143;
			public static final Distance HARD_MAX_HEIGHT = Feet.of(2); // Feet
			public static final double ABSOLUTE_ENCODER_GEAR_RATIO = 19.901;
			public static final Distance HEIGHT_OFFSET = Meter.of(0.3);

		}
		public static final Distance TOLLERANCE = Feet.of(0.2); // Feet
		public static final Distance ZERO_HEIGHTS_ABOVE_BASE = Feet.of(0.520); // Feet
	}

	public static class ArmConstants {
		public static class Shoulder {
			public static final boolean ENABLED = true;
			public static final int ID = 4;
			public static final double MAX_VELOCITY = 150;//120; // Not sure the unit
			public static final double MAX_ACCELERATION = 55;//200; // Not sure the unit
			public static final double P = 0.135; //0.7;
			public static final double I = 0;
			public static final double D = 0.009;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage G = Volt.of(0.5);//3.55); // Volts
			public static final Voltage V = Volt.of(0.13); // Volts/(Degrees/Second)
			public static final Voltage A = Volt.of(0.03); // Volts/(Degrees/Second^2)
			public static final Mass MASS = Pound.of(35); // Pounds (Including the wrist)
			public static final Angle MIN_ANGLE = Degree.of(-80.0); // Degrees
			public static final Angle MAX_ANGLE = Degree.of(70.0); // Degrees
			public static final Angle ABSOLUTE_ENCODER_OFFSET = Degree.of(0); // Degrees
			public static final double GEAR_RATIO = 48;
			public static final double ABSOLUTE_ENCODER_GEAR_RATIO = 1;
			public static final Angle ABSOLUTE_ENCODER_PUSH_BACK = Degree.of(25.0); // Degrees
			public static final Distance CENTER_OFFSET_FORWARD = Feet.of(0.492126); // Feet
			public static final Distance STAGE_OFFSET_UP = Feet.of(0.958); // Feet
			public static final Angle TOLLERANCE = Degree.of(3.0); // Degrees
		}

		public static class Wrist {
			public static final int ID = 5;
			public static final double MAX_VELOCITY = 120; // Not sure the unit
			public static final double MAX_ACCELERATION = 100; // Not sure the unit
			public static final double P = 0.7;
			public static final double I = 0;
			public static final double D = 0.05;
			public static final Voltage S = Volt.of(0.05); // Volts
			public static final Voltage V = Volt.of(0.05); // Volts/(Degrees/Second)
			public static final Voltage A = Volt.of(0.02); // Volts/(Degrees/Second^2)
			public static final Mass MASS = Pound.of(15.0); // Pounds
			public static final double MOI = 0.05; // Moment of innertia jKgMetersSquared
			public static final double GEAR_RATIO = 400;
			public static final Angle TOLLERANCE = Degree.of(2); // Degrees
		}

		public static class Intake {
			public static final int ID = 6;

			public static class Simulation {
				public static final Distance WIDTH = Feet.of(1.05); // Feet
				public static final Distance LENGTH = Feet.of(0.7); // Feet
				public static final Distance HEIGHT = Feet.of(0.65); // Feet
				public static final LinearVelocity OUTTAKE_VELOCITY = FeetPerSecond.of(1); // Feet/Second
			}
		}

		public static class IntakeSensor {
			public static final int ID = 0;
		}

		public static final Distance MINIMUM_HEIGHT_IN_BUMPER = Feet.of(0.7); // Feet
		public static final Angle OUT_BUMPER_ANGLE = Degree.of(-60); // Degree
		public static final Distance OUT_BUMPER_OFFSET = Feet.of(0.01);
		public static final Distance LENGTH = Feet.of(2.2); // Feet

	}

	public static class ClimbConstants {
		public static final int ID = 7;
	}

	public static class SetpointConstants {
		public static class GroundIntake {
			public static final Distance ELEVATOR_GROUND_HEIGHT = Meter.of(0); // Feet
			public static final Angle WRIST_ANGLE = Degree.of(0.0); // Degrees
			public static final Angle SHOULDER_ANGLE = Degree.of(-45.0); // Degrees
		}
	}

	public static class ReefConstants {
		public static class FieldConstants {
			public static final int[] BLUE_ALLIANCE_REEF_TAG_IDS = {21, 22, 17, 18, 19, 20};
			public static final int[] RED_ALLIANCE_REEF_TAG_IDS = {10, 9, 8, 7, 6, 11};

			public static class L1 {
				public static final Distance MAX_HEIGHT = Feet.of(1.9); // Feet
				public static final Angle BRANCH_ANGLE = Degree.of(-30.0); // Degrees
				public static final Angle WRIST_ANGLE = Degree.of(0); // Degrees
			}

			public static class L2 {
				public static final Distance MAX_HEIGHT = Feet.of(2.65748); // Feet
				public static final Angle BRANCH_ANGLE = Degree.of(35.0); // Degrees
				public static final Angle WRIST_ANGLE = Degree.of(90); // Degrees
			}

			public static class L3 {
				public static final Distance MAX_HEIGHT = Feet.of(3.96982); // Feet
				public static final Angle BRANCH_ANGLE = Degree.of(35.0); // Degrees
				public static final Angle WRIST_ANGLE = Degree.of(90); // Degrees
			}
			
			public static class L4 {
				public static final Distance MAX_HEIGHT = Feet.of(5.99); // Feet
				public static final Angle BRANCH_ANGLE = Degree.of(90.0); // Degrees
				public static final Angle WRIST_ANGLE = Degree.of(90); // Degrees
			}

			public static final Distance BRANCH_LEFT_OFFSET = Feet.of(0.563040616798); // Feet
			public static final Distance BRANCH_FORWARD_OFFSET = Feet.of(0.21141732); // Feet
		}

		public static final Angle LIFT_ANGLE = Degree.of(25.0);
		public static final Distance CLOSE_DISTANCE = Feet.of(13.0); // Feet
		public static final Distance SCORING_OFFSET = Feet.of(0.05); // Feet
	}

	public static class CoralStationConstants {
		public static class FieldConstants {
		  public static final int[] BLUE_ALLIANCE_CORAL_STATION_TAG_IDS = {13, 12}; // Left, Right
		  public static final int[] RED_ALLIANCE_CORAL_STATION_TAG_IDS = {1, 2}; // Left, Right
		}
	
		public static final Distance RIGHT_OFFSET = Feet.of(-2.0); // Feet (For the right alliance coral station)
		public static final Distance FORWARD_OFFSET = Feet.of(-0.19); // Feet
		public static final Distance VERTICAL_OFFSET = Feet.of(3.25); // Feet (From the carpet)
		public static final Angle PICK_UP_ANGLE = Degree.of(45.0); // Degrees
	  }

	public static class AutoConstants {
		public static final LinearVelocity TRANSLATION_MAX_VELOCITY = FeetPerSecond.of(12.0);
		public static final LinearAcceleration TRANSLATION_MAX_ACCELERATION = FeetPerSecondPerSecond.of(7.0);
		public static final AngularVelocity ROTATION_MAX_VELOCITY = DegreesPerSecond.of(180.0);
		public static final AngularAcceleration ROTATION_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(180.0);
		public static final double TRANSLATION_FEEDFORWARD_DIVISOR = 2;
		public static final double ROTATION_FEEDFORWARD_DIVISOR = 4;
	}

	public static class VisionConstants {
		public static final Matrix<N3, N1> VISION_SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.6); // The standard deviations of our vision estimated poses, which affect correction rate
		public static final Matrix<N3, N1> VISION_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.3); // The standard deviations of our vision estimated poses, which affect correction rate
		public static final double TARGET_DISTANCE_STD_DEVS_DIVISOR = 30; // The higher this is the less that far targets increase the std devs
		public static final double TARGET_AMBIGUITY_STD_DEVS_DIVISOR = 5; // The higher this is the less that a high average ambiguty increases the std devs
		public static final double CUTOFF = 2.5;

		public static class Limelight_4 {
			public static final String NAME = "Limelight 4";
			public static final boolean ENABLED = false;
			public static final Distance FRONT_OFFSET = Feet.of(0.0); // Feet
			public static final Distance LEFT_OFFSET = Feet.of(-1.31234); // Feet
			public static final Distance HEIGHT_OFFSET = Feet.of(0.95276); // Feet
			public static final Angle ROLL = Degree.of(0); // Degrees
			public static final Angle PITCH = Degree.of(0); // Degrees
			public static final Angle YAW = Degree.of(0); // Degrees
			public static final Distance EFFECTIVE_RANGE = Feet.of(16.4042); // Feet

			public static class CameraProperties {
				public static final int WIDTH = 1280; // Pixels
				public static final int HEIGHT = 800; // Pixels
				public static final int FPS = 120;
				public static final Angle DIAGONAL_FOV = Degree.of(91.12); // Degrees
				public static final double AVERGAGE_PIXEL_ERROR = 0.25;
				public static final double AVERGAGE_PIXEL_ERROR_STD_DEVS = 0.06;
				public static final double AVERAGE_LATENCY = 15; // Miliseconds
				public static final double AVERAGE_LATENCY_STD_DEVS = 0.1;
			}
		}
	}

	public static class SimulationConstants {
		public static class StartingSpawnCoral {
		  public static final boolean ENABLED = false;
		  public static final int SPAWN_COUNT = 24;
		  public static final Distance SPAWN_X = Feet.of(5); // Feet
		  public static final Distance SPAWN_Y = Feet.of(5); // Feet
		  public static final Distance SPAWN_RADIUS = Feet.of(3); // Feet
		}
	
		public static class CoralStations {
		  public static final boolean ENABLED = true;
		  public static final Distance FORWARD_OFFSET = Feet.of(-0.2); // Feet
		  public static final Distance VERTICAL_OFFSET = Feet.of(-1.49); // Feet
		  public static final Distance WIDTH = Feet.of(6.354); // Feet
		  public static final Distance LENGTH = Feet.of(0.5); // Feet
		  public static final Distance HEIGHT = Feet.of(0.7); // Feet
		  public static final Angle RAMP_ANGLE = Degree.of(35.0); // Degree
		  public static final LinearVelocity SPAWN_VELOCITY = FeetPerSecond.of(3);
		  public static final double ANGLE_DIFFERENCE_THRESHOLD = 0.05;
		  public static final Angle WRIST_DIFFERENCE_THRESHOLD = Degree.of(20); // Degree
		  public static final Time IN_ZONE_TIME = Second.of(0.5); // Second
		}
		
		public static final boolean SIMULATE_SHOULDER_OFFSET = false;
		public static final boolean SIMULATE_ELEVATOR_OFFSET = false;
		public static final Mass CORAL_WEIGHT = Pound.of(1.4); // Pound
		public static final boolean ASSUME_START_WITH_CORAL = true;
	  }

	public static class DriverConstants {
		public static final int PORT = 0;
		public static final double DEADBAND = 0.12;
		public static final double TRANSLATION_SCALE = 0.5;
		public static final double ROTATION_SCALE = 0.7;
		public static final double LEFT_JOYSTICK_EXPONENT = 1.25;
		public static final double RIGHT_JOYSTICK_EXPONENT = 1.25;
		public static final double TRIGGER_EXPONENT = 1;
		public static final LinearVelocity CONTROL_ELEVATOR_SPEED = FeetPerSecond.of(1); // Feet/Second
		public static final AngularVelocity CONTROL_SHOULDER_SPEED = DegreesPerSecond.of(50); // Degrees/Second
		public static final AngularVelocity BABY_CONTROL_SHOULDER_LIMIT_SPEED = DegreesPerSecond.of(10); // Degrees/Second
		public static final AngularVelocity CONTROL_WRIST_SPEED = DegreesPerSecond.of(50); // Degrees/Second
		public static final AngularVelocity BABY_CONTROL_WRIST_LIMIT_SPEED = DegreesPerSecond.of(10); // Degrees/Second
		public static final double INTAKE_SPEED = -0.9; // Percent
		public static final double OUTTAKE_SPEED = 0.9; // Percent
		public static final double CLIMB_UP_SPEED = -1; // Percent
	}

	public static class TelemetryConstants {
		public static boolean VISION_TELEMETRY = true;
		public static boolean ELEVATOR_TELEMETRY = true;
		public static boolean SHOULDER_TELEMETRY = true;
		public static boolean WRIST_TELEMETRY = true;
		public static boolean END_EFFECTOR_TELEMETRY = true;
		public static boolean SIMULATION_TELEMETRY = false;
		public static boolean PATHFINDING_TELEMETRY = true;
		public static boolean DEBUG = false;
		public static boolean ANIMATIONS = false;
	}
}