package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.objects.InverseKinematicState;
import frc.robot.objects.InverseKinematicStatus;
import frc.robot.objects.RectangularPrism;
import frc.robot.utilities.math.PoseUtilities;

public class EndEffectorSubsystem extends SubsystemBase {
    private RectangularPrism Test = new RectangularPrism(new Pose3d(), Meter.of(1), Meter.of(0.5), Meter.of(0.75));

    public EndEffectorSubsystem() {

    }

    public Pose3d calculateEndEffectorPose() {
        // Calculate where the end effector of the arm is
        Pose3d endEffectorPose = new Pose3d(RobotContainer.swerveSubsystem.getPose())
                .plus(new Transform3d(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD.in(Meter), 0,
                        RobotContainer.elevatorSubsystem.getCarpetElevatorHeight().in(Meter),
                        new Rotation3d(RobotContainer.wristSubsystem.getWristAngle().in(Radian),
                                -RobotContainer.shoulderSubsystem.getShoulderAngle().in(Radian), 0)));

        // Add the length of the arm onto the end effector pose to get the positon at the end of the arm
        return endEffectorPose.plus(
                new Transform3d(Constants.ArmConstants.LENGTH.in(Meter), 0, 0, new Rotation3d()));
    }

    /**
     * 
     * @param endEffectorTarget The target pose for the end effector. It will try to respect the pitch and yaw
     * @param minimumRobotDistance The minimum distance bettween the robot center and the end effector in 2d space
     * @return The state of the robot to fufuil the above criteria
     */
    public InverseKinematicState calculateInverseKinematicState(Pose3d endEffectorTarget,
            Distance minimumRobotDistance) {
        InverseKinematicStatus targetStatus = InverseKinematicStatus.PERFECT;

        double targetAngle = endEffectorTarget.getRotation().getY();
        double armLength = Constants.ArmConstants.LENGTH.in(Meter) + Constants.ReefConstants.SCORING_OFFSET.in(Meter);

        // Calculate the horizontal distance between the robot center and the end effector when at the
        // angle needed to reach the target and account for the offset of the shoulder joint relative to the center of the robot
        double horizontalDistance = calculateHorizontalDistance(Radian.of(targetAngle), Meter.of(armLength)).in(Meter);

        // For some locations the size of the robots swerve base prevents it from getting close enough to achive an optimal angle
        if (minimumRobotDistance.in(Meter) > horizontalDistance) {
            targetStatus = InverseKinematicStatus.CLOSE;

            System.out.println("Invalid horizontal distance for auto score trying backup method 1");

            targetAngle = constrainedDistanceAngle(minimumRobotDistance, Radian.of(targetAngle), Meter.of(armLength))
                    .in(Radian);
        }

        // Calculate the vertical offset between the shoulder and the end effector when at the
        // angle needed to reach the target
        double verticalDistance = Math.sin(targetAngle) * armLength;

        // Using that verical offset calculate the height that the elevator has to be at
        double elevatorTargetHeight = endEffectorTarget.getZ() - verticalDistance;

        // For some locations the limits on the elevators height prevent it from achiving an optimal angle
        if (!RobotContainer.elevatorSubsystem.checkGlobalHeightPossible(Meter.of(elevatorTargetHeight))) {
            targetStatus = InverseKinematicStatus.CLOSE;

            System.out.println("Invalid elevator height for auto score trying backup method 2");

            targetAngle = constrainedHeightAngle(Meter.of(endEffectorTarget.getZ()), Meter.of(armLength)).in(Radian);
        }

        // Recalculate the horizontal distance because it could have been changed
        double finalHorizontalDistance = calculateHorizontalDistance(Radian.of(targetAngle), Meter.of(armLength)).in(Meter);

        double finalElevatorHeight = MathUtil.clamp(elevatorTargetHeight, 0, RobotContainer.elevatorSubsystem.getMaxGroundHeight().in(Meter));

        if (Double.isNaN(targetAngle) || Double.isNaN(finalHorizontalDistance) || Double.isNaN(finalElevatorHeight)) {
            targetStatus = InverseKinematicStatus.INVALID;
        }

        // Calculate the pose the robot has to be in to score the coral
        Pose2d chassisPose = endEffectorTarget.toPose2d().plus(new Transform2d(-finalHorizontalDistance, 0, new Rotation2d()));

        return new InverseKinematicState(chassisPose, Meter.of(elevatorTargetHeight), Radian.of(targetAngle), targetStatus);
    }


    private Distance calculateHorizontalDistance(Angle angle, Distance armLength) {
        return Meter.of(Math.cos(angle.in(Radian)) * armLength.in(Meter))
                .plus(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);
    }


    private Angle constrainedDistanceAngle(Distance minimumRobotDistance, Angle angle, Distance armLength) {
        // The minimum distance does not account for the offset of the shoulder joint so it is subtracted to get the minimum distance bettween the shoulder and the target
        double horizontalRobotArmDistance = minimumRobotDistance.in(Meter)
                - Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD.in(Meter);

        // Compute the angle to reach the point while taking into account the minimum distance
        double newTargetAngle = Math.acos(horizontalRobotArmDistance / armLength.in(Meter));

        newTargetAngle *= Math.signum(angle.in(Radian));

        return Radian.of(newTargetAngle);
    }


    private Angle constrainedHeightAngle(Distance targetHeight, Distance armLength) {
        double maxElevatorHeight = RobotContainer.elevatorSubsystem.getMaxHeight().in(Meter);

        // This asumes that the requested height is not within the elevators range of motion
        // It subtracts the pivot point offset to get the vertical distance between the lowest bound on the elevator
        // and the target
        double targetToPivotOffset = targetHeight.in(Meter)
                - RobotContainer.elevatorSubsystem.getPivotPointOffset(true).in(Meter);

        // If the offset is higher then the max height of the elevator the target must be higher then the max height of the elevatora
        // This code changes it so it is now the offset from the max travel of the elevator to the branch
        if (targetToPivotOffset >= maxElevatorHeight) {
            targetToPivotOffset -= maxElevatorHeight;
        }

        return Radian.of(Math.asin(targetToPivotOffset / armLength.in(Meter)));
    }


    @Override
    public void periodic() {
        Pose3d endEffPose = calculateEndEffectorPose();

        Test.changeCenterPose(endEffPose);

        Pose3d Point = new Pose3d(5, 5, 1, new Rotation3d());

        SmartDashboard.putNumberArray("TESTBOX", Test.generateDebugWireframe());

        boolean isC = Test.poseInside(Point);

        SmartDashboard.putNumberArray("TESTPOINT", PoseUtilities.convertPoseToNumbers(Point));
        SmartDashboard.putBoolean("TESTCOLLIDE", isC);
    }
}
