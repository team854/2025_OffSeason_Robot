package frc.robot.objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * 
 * InverseKinematicsState holds the state of the robot to reach an end effector pose
 * @param chassisPose The pose that the robots chassis must be at
 * @param elevatorHeight The height of the elevator from the ground
 * @param shoulderAngle The angle of the shoulder with 0 being parallel to the ground
 * @param status If a solution was found and if it is perfect
 */
public record InverseKinematicState(Pose2d chassisPose,
            Distance elevatorHeight,
            Angle shoulderAngle,
            InverseKinematicStatus status) {
    
}
