package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.objects.InverseKinematicState;
import frc.robot.objects.InverseKinematicStatus;
import frc.robot.objects.RectangularPrism;
import frc.robot.subsystems.simulation.ClawSimulation;
import frc.robot.utilities.math.PoseUtilities;

public class EndEffectorSubsystem extends SubsystemBase {
    /*
     * Constants
     */
    private final Quaternion ninetyZRotation = new Quaternion(
            Math.cos(Units.degreesToRadians(90) / 2),
            0,
            0,
            Math.sin(Units.degreesToRadians(90) / 2)
        ); // Thanks claude 3.7 xD

    /*
     * Motor
     */
    private final VictorSPX intakeMotor = new WPI_VictorSPX(Constants.ArmConstants.Intake.ID);

    /*
     * Sensor
     */
    private final DigitalInput intakeSensor = new DigitalInput(Constants.ArmConstants.IntakeSensor.ID);

    /*
     * Simulation
     */
    private final boolean isSimulation = Robot.isSimulation();
    private ClawSimulation clawSimulation;
    
    public EndEffectorSubsystem() {

        if (Robot.isSimulation()) {
            System.out.println("Creating end effector simulation");
            clawSimulation = new ClawSimulation(new RectangularPrism(new Pose3d(), Constants.ArmConstants.Intake.Simulation.WIDTH, Constants.ArmConstants.Intake.Simulation.LENGTH, Constants.ArmConstants.Intake.Simulation.HEIGHT), 1);

            clawSimulation.setActiveStage(true);
        }

        System.out.println("Created EndEffectorSubsystem");
    }

    public Pose3d calculateEndEffectorPose(Pose3d robotPose) {
        // Calculate where the end effector of the arm is
        Pose3d endEffectorPose = robotPose
                .plus(new Transform3d(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD.in(Meter), 0,
                        RobotContainer.elevatorSubsystem.getOverallHeight().in(Meter),
                        new Rotation3d(RobotContainer.wristSubsystem.getWristAngle().in(Radian),
                                -RobotContainer.shoulderSubsystem.getShoulderAngle().in(Radian), 0)));

        // Add the length of the arm onto the end effector pose to get the positon at the end of the arm
        return endEffectorPose.plus(
                new Transform3d(Constants.ArmConstants.LENGTH.in(Meter), 0, 0, new Rotation3d()));
    }

    public Pose3d calculateEndEffectorPose() {
        return calculateEndEffectorPose(new Pose3d(RobotContainer.swerveSubsystem.getPose()));
    }

    public Pose3d calculateCoralPose(Pose3d robotPose) {
        Pose3d endEffectorPose = calculateEndEffectorPose(robotPose);
        
        return new Pose3d(endEffectorPose.getTranslation(), new Rotation3d(endEffectorPose.getRotation().getQuaternion().times(ninetyZRotation)));
    }

    public Pose3d calculateCoralPose() {
        return calculateCoralPose(new Pose3d(RobotContainer.swerveSubsystem.getPose()));
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
        if (!RobotContainer.elevatorSubsystem.checkOverallHeightPossible(Meter.of(elevatorTargetHeight))) {
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

    public boolean hasCoral() {
        if (isSimulation) {
            return clawSimulation.getGamePieceCount() > 0;
        } else {
            // The intake sensor is inverted so an ON signal means there is no game peice
            return !intakeSensor.get();
        }
     }

    public void setIntakeSpeed(double intakeSpeed) {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, intakeSpeed);
    }

    public double getIntakeSpeedPercent() {
        return intakeMotor.getMotorOutputPercent();
    }

    public double getIntakeAppliedVoltage() {
        return intakeMotor.getMotorOutputVoltage();
    }

    public Command setIntakeSpeedCommand(double intakeSpeed) {
        return new StartEndCommand(() -> setIntakeSpeed(intakeSpeed), () -> setIntakeSpeed(0), this);
    }

    public Command intakeUntil(double intakeSpeed, boolean desiredState, double timeoutSeconds) {
        return setIntakeSpeedCommand(intakeSpeed).until(() -> hasCoral() == desiredState).withTimeout(timeoutSeconds);
    }

    public ClawSimulation getClawSimulation() {
        return clawSimulation;
    }

    @Override
    public void simulationPeriodic() {
        // Calculate the end effector pose using the actual pose from the simulation
        Pose3d endEffPose = calculateEndEffectorPose(new Pose3d(RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get()));

        // Iterate on the claw simulatioh
        clawSimulation.setActiveStage(getIntakeSpeedPercent() < -0.2);
        clawSimulation.setPickupVolumeCenterPose(endEffPose);
        clawSimulation.iterate();

        // If its outtaking then try to eject the coral
        if (getIntakeSpeedPercent() > 0.2) {
            clawSimulation.ejectGamePiece();
        }

        SmartDashboard.putNumberArray("SIGH", clawSimulation.getPickupVolume().generateDebugWireframe());
    }

    @Override
    public void periodic() {
        
    }
}
