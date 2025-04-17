package frc.robot.objects;

import static edu.wpi.first.units.Units.Meter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utilities.math.PoseUtilities;

public class RectangularPrism {
    private Pose3d centerPose;
    private double width;
    private double length;
    private double height;

    public RectangularPrism(Pose3d centerPose, Distance width, Distance length, Distance height) {
        this.centerPose = centerPose;
        this.width = width.in(Meter);
        this.length = length.in(Meter);
        this.height = height.in(Meter);
    }

	public void changeCenterPose(Pose3d centerPose) {
		this.centerPose = centerPose;
	}

    public boolean poseInside(Pose3d testPose) {
        Translation3d poseTranslation = testPose.getTranslation();

        poseTranslation = poseTranslation.minus(this.centerPose.getTranslation()).rotateBy(this.centerPose.getRotation().unaryMinus());

        return Math.abs(poseTranslation.getX()) < (this.length / 2) &&
                Math.abs(poseTranslation.getY()) < (this.width / 2) &&
                Math.abs(poseTranslation.getZ()) < (this.height / 2);
    }

	public Double[] generateDebugWireframe() {
		Pose3d topLeftFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / 2, this.height / 2, new Rotation3d()));
		Pose3d topRightFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / -2, this.height / 2, new Rotation3d()));
		Pose3d topLeftBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / 2, this.height / 2, new Rotation3d()));
		Pose3d topRightBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / -2, this.height / 2, new Rotation3d()));
		Pose3d bottomLeftFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / 2, this.height / -2, new Rotation3d()));
		Pose3d bottomRightFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / -2, this.height / -2, new Rotation3d()));
		Pose3d bottomLeftBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / 2, this.height / -2, new Rotation3d()));
		Pose3d bottomRightBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / -2, this.height / -2, new Rotation3d()));

		/*/
		Pose3d[] outputPoses = {
			topLeftFront, topRightFront,
			topLeftFront, topLeftBack,
			topRightBack, topLeftBack,
			topRightBack, topRightFront,
			
			bottomRightBack, bottomRightFront,
			bottomLeftFront, bottomRightFront,
			bottomLeftFront, bottomLeftBack,
			bottomRightBack, bottomLeftBack,
					
			topLeftFront, bottomLeftFront,
			topRightFront, bottomRightFront,
			topLeftBack, bottomLeftBack,
			topRightBack, bottomLeftBack};
		*/
		Pose3d[] outputPoses = {
			topLeftFront, topRightFront, topRightBack, topLeftBack, topLeftFront, bottomLeftFront, bottomRightFront, topRightFront, bottomRightFront, bottomRightBack, topRightBack, bottomRightBack, bottomLeftBack, topLeftBack, bottomLeftBack, bottomLeftFront
		};

		List<Double> outputArray = new ArrayList<>();
		for (Pose3d pose : outputPoses) {
			double[] poseDoubleList = PoseUtilities.convertPoseToNumbers(pose);
			for (double num : poseDoubleList) {
				outputArray.add(num);
			}
		}
		return outputArray.toArray(new Double[0]);
	}
}
