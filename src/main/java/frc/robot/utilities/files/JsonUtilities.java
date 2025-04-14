package frc.robot.utilities.files;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.utilities.text.TextUtilities;

public class JsonUtilities {
    public static String toJson(Object serializableObject) {

        try {
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();

            return ow.writeValueAsString(extractObjectFields(serializableObject));
        } catch (Exception e) {
            e.printStackTrace();
        }
        return "{}";
    }

    public static Map<String, Object> fromJson(String str) {
        try {
            return new ObjectMapper().readValue(str, new TypeReference<LinkedHashMap<String, Object>>() {});
        } catch (Exception e) {
            e.printStackTrace();
            return new LinkedHashMap<>();
        }
    }

    private static Map<String, Object> extractObjectFields(Object objectInstance) {
        Map<String, Object> outputMap = new HashMap<String, Object>();

        for (Field objectField : objectInstance.getClass().getDeclaredFields()) {
            try {
                outputMap.put(makeFirstLetterLower(objectField.getName()), objectField.get(objectInstance));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        for (Class<?> objectClass : objectInstance.getClass().getDeclaredClasses()) {
            try {
                Object instanceClass = objectClass.getDeclaredConstructor().newInstance();
                outputMap.put(makeFirstLetterLower(objectClass.getSimpleName()), extractObjectFields(instanceClass));
            } catch (Exception e) {
                e.printStackTrace();
            }

        }

        return outputMap;
    }

    private static String makeFirstLetterLower(String str) {
        return str.substring(0, 1).toLowerCase() + str.substring(1);
    }

    public static Class<?> getInnerClass(Class<?> parentClass, String className) {
        try {
            return Class.forName(parentClass.getName() + "$" + className);
        } catch (ClassNotFoundException e) {
            return null;
        }
    }

    // Writen by claude 3.7 sonnet then modified to be cleaner.
    public static String moduleToJson(Class<?> moduleClass) {
        try {
            Map<String, Object> outputMap = new LinkedHashMap<String, Object>();

            Class<?> locationClass = getInnerClass(moduleClass, "Location");
            Class<?> driveClass = getInnerClass(moduleClass, "DriveMotor");
            Class<?> angleClass = getInnerClass(moduleClass, "AngleMotor");
            Class<?> encoderClass = getInnerClass(moduleClass, "Encoder");

            // Get the encoder offset
            outputMap.put("absoluteEncoderOffset",((Angle) moduleClass.getDeclaredField("ABSOLUTE_ENCODER_OFFSET").get(null)).in(Degree));

            // Get location data
            Map<String, Object> location = new LinkedHashMap<String, Object>();
            location.put("front", ((Distance) locationClass.getDeclaredField("FRONT").get(null)).in(Inch));
            location.put("left", ((Distance) locationClass.getDeclaredField("LEFT").get(null)).in(Inch));
            outputMap.put("location", location);

            // Get drive data
            Map<String, Object> drive = new LinkedHashMap<String, Object>();
            drive.put("type", driveClass.getDeclaredField("TYPE").get(null));
            drive.put("id", driveClass.getDeclaredField("ID").get(null));
            drive.put("canbus", driveClass.getDeclaredField("CANBUS").get(null));
            outputMap.put("drive", drive);

            // Get angle data
            Map<String, Object> angle = new LinkedHashMap<String, Object>();
            angle.put("type", angleClass.getDeclaredField("TYPE").get(null));
            angle.put("id", angleClass.getDeclaredField("ID").get(null));
            angle.put("canbus", angleClass.getDeclaredField("CANBUS").get(null));
            outputMap.put("angle", angle);

            // Get encoder data
            Map<String, Object> encoder = new LinkedHashMap<String, Object>();
            encoder.put("type", encoderClass.getDeclaredField("TYPE").get(null));
            encoder.put("id", encoderClass.getDeclaredField("ID").get(null));
            encoder.put("canbus", encoderClass.getDeclaredField("CANBUS").get(null));
            outputMap.put("encoder", encoder);

            // Get inverted data
            Map<String, Object> inverted = new LinkedHashMap<String, Object>();
            inverted.put("drive", driveClass.getDeclaredField("INVERTED").get(null));
            inverted.put("angle", angleClass.getDeclaredField("INVERTED").get(null));
            outputMap.put("inverted", inverted);

            // Get encoder inverted
            outputMap.put("absoluteEncoderInverted", encoderClass.getDeclaredField("INVERTED").get(null));

            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(outputMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }

    public static String physicalPropertiesToJson() {
        try {
            Map<String, Object> outputMap = new LinkedHashMap<String, Object>();

            outputMap.put("robotMass", Constants.RobotKinematicConstants.MASS.in(Pound));

            // Get ramp rate data
            Map<String, Object> rampRate = new LinkedHashMap<String, Object>();
            rampRate.put("angle", Constants.SwerveConstants.MotorConstants.Angle.RAMP_RATE.in(Second));
            rampRate.put("drive", Constants.SwerveConstants.MotorConstants.Drive.RAMP_RATE.in(Second));
            outputMap.put("rampRate", rampRate);

            // Get conversion factor data
            Map<String, Object> conversionFactors = new LinkedHashMap<String, Object>();

            Map<String, Object> angle = new LinkedHashMap<String, Object>();
            angle.put("gearRatio", Constants.SwerveConstants.MotorConstants.Angle.GEAR_RATIO);
            angle.put("factor", Constants.SwerveConstants.MotorConstants.Angle.FACTOR);

            Map<String, Object> drive = new LinkedHashMap<String, Object>();
            drive.put("gearRatio", Constants.SwerveConstants.MotorConstants.Drive.GEAR_RATIO);
            drive.put("factor", Constants.SwerveConstants.MotorConstants.Drive.FACTOR);
            drive.put("diameter", Constants.SwerveConstants.WheelConstants.DIAMETER.in(Inch));

            conversionFactors.put("angle", angle);
            conversionFactors.put("drive", drive);
            outputMap.put("conversionFactors", conversionFactors);

            outputMap.put("wheelGripCoefficientOfFriction", Constants.SwerveConstants.WheelConstants.WHEEL_GRIP_COF);
            outputMap.put("optimalVoltage", Constants.SwerveConstants.MotorConstants.OPTIMAL_VOLTAGE.in(Volt));

            // Get current limit data
            Map<String, Object> currentLimit = new LinkedHashMap<String, Object>();
            currentLimit.put("angle", Constants.SwerveConstants.MotorConstants.Angle.CURRENT_LIMIT.in(Amp));
            currentLimit.put("drive", Constants.SwerveConstants.MotorConstants.Drive.CURRENT_LIMIT.in(Amp));
            outputMap.put("currentLimit", currentLimit);
            
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(outputMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }

    public static String swerveDriveToJson() {
        try {
            Map<String, Object> outputMap = new LinkedHashMap<String, Object>();
            
            outputMap.put("invertedIMU", Constants.SwerveConstants.Imu.INVERTED);

            // Get imu data
            Map<String, Object> imu = new LinkedHashMap<String, Object>();
            imu.put("type", Constants.SwerveConstants.Imu.TYPE);
            imu.put("id", Constants.SwerveConstants.Imu.ID);
            imu.put("canbus", Constants.SwerveConstants.Imu.CANBUS);
            outputMap.put("imu", imu);

            outputMap.put("modules", Constants.SwerveConstants.MODULE_FILES);
            
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(outputMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }

    public static String pidfPropertiesToJson() {
        try {
            Map<String, Object> outputMap = new LinkedHashMap<String, Object>();

            // Get angle motor pid data
            Map<String, Object> angle = new LinkedHashMap<String, Object>();
            angle.put("p", Constants.SwerveConstants.MotorConstants.Angle.Pidf.P);
            angle.put("i", Constants.SwerveConstants.MotorConstants.Angle.Pidf.I);
            angle.put("d", Constants.SwerveConstants.MotorConstants.Angle.Pidf.D);
            angle.put("f", Constants.SwerveConstants.MotorConstants.Angle.Pidf.F);
            angle.put("iz", Constants.SwerveConstants.MotorConstants.Angle.Pidf.IZ);
            outputMap.put("angle", angle);

            // Get drive motor pid data
            Map<String, Object> drive = new LinkedHashMap<String, Object>();
            drive.put("p", Constants.SwerveConstants.MotorConstants.Drive.Pidf.P);
            drive.put("i", Constants.SwerveConstants.MotorConstants.Drive.Pidf.I);
            drive.put("d", Constants.SwerveConstants.MotorConstants.Drive.Pidf.D);
            drive.put("f", Constants.SwerveConstants.MotorConstants.Drive.Pidf.F);
            drive.put("iz", Constants.SwerveConstants.MotorConstants.Drive.Pidf.IZ);
            outputMap.put("drive", drive);
            
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(outputMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }

    public static String controllerPropertiesToJson() {
        try {
            Map<String, Object> outputMap = new LinkedHashMap<String, Object>();

            outputMap.put("angleJoystickRadiusDeadband", Constants.SwerveConstants.ANGLE_JOYSTICK_RADIUS_DEADBAND);

            // Get heading pid data
            Map<String, Object> heading = new LinkedHashMap<String, Object>();
            heading.put("p", Constants.SwerveConstants.HeadingPID.P);
            heading.put("i", Constants.SwerveConstants.HeadingPID.I);
            heading.put("d", Constants.SwerveConstants.HeadingPID.D);
            outputMap.put("heading", heading);
            
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(outputMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }

    public static String pathPlannerToJson(Map<String, Object> currentMap, List<Class<?>> swerveModules) {
        try {
            currentMap.put("robotWidth", Constants.RobotKinematicConstants.WIDTH.in(Meter));
            currentMap.put("robotLength", Constants.RobotKinematicConstants.LENGTH.in(Meter));
            currentMap.put("holonomicMode", true);
            currentMap.put("robotMass", Constants.RobotKinematicConstants.MASS.in(Kilogram));
            currentMap.put("robotMOI", Constants.RobotKinematicConstants.MOI);
            currentMap.put("maxDriveSpeed", Constants.RobotKinematicConstants.MAX_ACHIEVABLE_SPEED.in(MetersPerSecond));
            currentMap.put("driveMotorType", Constants.RobotKinematicConstants.DRIVE_MOTOR_TYPE);
            currentMap.put("driveCurrentLimit", Constants.SwerveConstants.MotorConstants.Drive.CURRENT_LIMIT.in(Amp));
            currentMap.put("wheelCOF", Constants.SwerveConstants.WheelConstants.WHEEL_GRIP_COF);
            currentMap.put("driveWheelRadius", Constants.SwerveConstants.WheelConstants.DIAMETER.in(Meter) / 2);
            currentMap.put("driveGearing", Constants.SwerveConstants.MotorConstants.Drive.GEAR_RATIO);
            currentMap.put("bumperOffsetX", Constants.RobotKinematicConstants.BumperOffset.X.in(Meter));
            currentMap.put("bumperOffsetY", Constants.RobotKinematicConstants.BumperOffset.Y.in(Meter));

            for (Class<?> moduleClass : swerveModules) {
                String paramPrefix = TextUtilities.removeLowerCase(moduleClass.getSimpleName()).toLowerCase();

                Class<?> locationClass = getInnerClass(moduleClass, "Location");

                currentMap.put(paramPrefix + "ModuleX", ((Distance) locationClass.getDeclaredField("FRONT").get(null)).in(Meter)); 
                currentMap.put(paramPrefix + "ModuleY", ((Distance) locationClass.getDeclaredField("LEFT").get(null)).in(Meter));
            }
            
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            return ow.writeValueAsString(currentMap);
        } catch (Exception e) {
            e.printStackTrace();
            return "{}";
        }
    }
}
