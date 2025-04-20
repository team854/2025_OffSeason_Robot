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
import frc.robot.utilities.math.VectorUtilities;

public class RectangularPrism {
    private Pose3d centerPose;
    private double width;
    private double length;
    private double height;
	private double maxDistance;

    public RectangularPrism(Pose3d centerPose, Distance width, Distance length, Distance height) {
        this.centerPose = centerPose;
        this.width = width.in(Meter);
        this.length = length.in(Meter);
        this.height = height.in(Meter);
		this.maxDistance = Math.sqrt(Math.pow(this.width, 2) + Math.pow(this.length, 2) + Math.pow(this.height, 2));
    }

	public double getWidth() {
		return this.width;
	}

	public double getLength() {
		return this.length;
	}

	public double getHeight() {
		return this.height;
	}

	public double getMaxDistance() {
		return this.maxDistance;
	}
 
	public Pose3d getCenterPose() {
		return this.centerPose;
	}

	public void changeCenterPose(Pose3d centerPose) {
		this.centerPose = centerPose;
	}

    public boolean poseInside(Pose3d testPose) {
		if (PoseUtilities.calculatePoseDistance(testPose, this.centerPose).in(Meter) > (this.maxDistance / 2)) {
			return false;
		}

        Translation3d poseTranslation = testPose.getTranslation();

        poseTranslation = poseTranslation.minus(this.centerPose.getTranslation()).rotateBy(this.centerPose.getRotation().unaryMinus());

        return Math.abs(poseTranslation.getX()) < (this.length / 2) &&
                Math.abs(poseTranslation.getY()) < (this.width / 2) &&
                Math.abs(poseTranslation.getZ()) < (this.height / 2);
    }
 
	public boolean checkOverlap(RectangularPrism testRect) {
		Translation3d centerPointDiff = testRect.centerPose.getTranslation().minus(this.centerPose.getTranslation());

		if (centerPointDiff.getDistance(new Translation3d()) > ((this.getMaxDistance() + testRect.getMaxDistance()) / 2)) {
			return false;
		}

		Rotation3d thisRotation = this.centerPose.getRotation();
		Rotation3d testRotation = testRect.centerPose.getRotation();

		// Normalized vectors of each shape's x y and z axis
		Translation3d[] shapeAxis = {
			new Translation3d(1, 0, 0).rotateBy(thisRotation),
            new Translation3d(0, 1, 0).rotateBy(thisRotation),
            new Translation3d(0, 0, 1).rotateBy(thisRotation),
			new Translation3d(1, 0, 0).rotateBy(testRotation),
            new Translation3d(0, 1, 0).rotateBy(testRotation),
            new Translation3d(0, 0, 1).rotateBy(testRotation)
		};

		// Half of each shapes length, width, and height
		double[] halfExtents = {
			this.getLength() / 2,
			this.getWidth() / 2,
			this.getHeight() / 2,
			testRect.getLength() / 2,
			testRect.getWidth() / 2,
			testRect.getHeight() / 2
		};

		for (int index = 0; index < 6; index++) {
			if (!checkOverlapOnAxis(centerPointDiff, shapeAxis[index], shapeAxis, halfExtents)) {
				return false;
			}
		}

		// This helps handle edge to edge collsions
		for (int i = 0; i < 3; i++) {
            for (int j = 3; j < 6; j++) {
				Translation3d crossAxis = VectorUtilities.crossProduct(shapeAxis[i], shapeAxis[j]);

				double crossMagnitude = crossAxis.getDistance(new Translation3d());

				// If the cross magnitude is really small it means the axis are parallel
				if (crossMagnitude < 1e-6) {
                    continue;
                }

				if (!checkOverlapOnAxis(centerPointDiff, crossAxis, shapeAxis, halfExtents)) {
					return false;
				}
			}
		}

		return true;
	}

	private boolean checkOverlapOnAxis(Translation3d centerPointDiff, Translation3d checkAxis, Translation3d[] shapeAxis, double[] halfExtents) {

		double centerProjection = Math.abs(VectorUtilities.dotProduct(centerPointDiff, checkAxis));

		double shapesProjection = 0;
		for (int index = 0; index < 6; index++) {
			shapesProjection += Math.abs(VectorUtilities.dotProduct(shapeAxis[index], checkAxis) * halfExtents[index]);
		}

		return centerProjection <= shapesProjection;

	}

	/**
	 * Generates a trajectory that when used in Advantage Scope shows the {@link RectangularPrism}
	 * @return The trajectory
	 */
	public Double[] generateDebugWireframe() {
		Pose3d topLeftFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / 2, this.height / 2, new Rotation3d()));
		Pose3d topRightFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / -2, this.height / 2, new Rotation3d()));
		Pose3d topLeftBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / 2, this.height / 2, new Rotation3d()));
		Pose3d topRightBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / -2, this.height / 2, new Rotation3d()));
		Pose3d bottomLeftFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / 2, this.height / -2, new Rotation3d()));
		Pose3d bottomRightFront = this.centerPose.transformBy(new Transform3d(this.length / 2, this.width / -2, this.height / -2, new Rotation3d()));
		Pose3d bottomLeftBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / 2, this.height / -2, new Rotation3d()));
		Pose3d bottomRightBack = this.centerPose.transformBy(new Transform3d(this.length / -2, this.width / -2, this.height / -2, new Rotation3d()));

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
