package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.records.RectangularPrism;
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
                        new Rotation3d(RobotContainer.wristSubsystem.getWristAngle().in(Radian), -RobotContainer.shoulderSubsystem.getShoulderAngle().in(Radian),0)));

        // Add the length of the arm onto the end effector pose to get the positon at the end of the arm
        return endEffectorPose.plus(
                new Transform3d(Constants.ArmConstants.LENGTH.in(Meter), 0, 0, new Rotation3d()));
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
