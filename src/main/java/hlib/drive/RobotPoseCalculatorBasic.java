package hlib.drive;

/**
 * A {@code RobotPoseCalculatorBasic} can calculate the {@code Pose} of a {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class RobotPoseCalculatorBasic implements RobotPoseCalculator {

	/**
	 * The width of the {@code Robot}.
	 */
	protected double robotWidth;

	/**
	 * Constructs a {@code RobotPoseCalculatorBasic}.
	 * 
	 * @param robotWidth
	 *            the width of the {@code Robot}
	 */
	public RobotPoseCalculatorBasic(double robotWidth) {
		this.robotWidth = robotWidth;
	}

	/**
	 * Returns the next {@code Pose} of a {@code Robot} when it moves by the specified displacements.
	 * 
	 * @param pose
	 *            the current {@code Pose} of a {@code Robot}
	 * @param leftDisplacement
	 *            the displacement of the left wheels of the {@code Robot}
	 * @param rightDisplacement
	 *            the displacement of the right wheels of the {@code Robot}
	 * @return the next {@code Pose} of a {@code Robot} when it moves by the specified displacements
	 */
	@Override
	public Pose nextPose(Pose pose, double leftDisplacement, double rightDisplacement) {
		if (pose != null) {
			Pose leftPose = leftPose(pose).move(leftDisplacement);
			Pose rightPose = rightPose(pose).move(rightDisplacement);
			double directionalAngle = Pose.normalize(leftPose.angleTo(rightPose) + Math.PI / 2);
			return new Pose((leftPose.x() + rightPose.x()) / 2, (leftPose.y() + rightPose.y()) / 2, directionalAngle);
		}
		return null;
	}

	/**
	 * Returns the displacements of the left and right wheels of a {@code Robot} when it moves between the specified
	 * {@code Pose}s.
	 * 
	 * @param from
	 *            the {@code Pose} of a {@code Robot} before it moves
	 * @param to
	 *            the {@code Pose} of a {@code Robot} after it moves
	 * @return the displacements of the left and right wheels of a {@code Robot} when it moves between the specified
	 *         {@code Pose}s
	 */
	@Override
	public double[] wheelDisplacements(Pose from, Pose to) {
		return new double[] { displacement(leftPose(from), leftPose(to)),
				displacement(rightPose(from), rightPose(to)) };
	}

	/**
	 * Returns the displacements of the left and right wheels of a {@code Robot} when it rotates by the specified angle.
	 * 
	 * @param angle
	 *            an angle (in radians)
	 * @param forwardOnly
	 *            a boolean value indicating whether or not only forward movement is allowed
	 * @return the displacements of the left and right wheels of a {@code Robot} when it rotates by the specified angle
	 */
	@Override
	public double[] wheelDisplacements(double angle, boolean forwardOnly) {
		angle = Pose.normalize(angle);
		double rightDisplacement = angle * this.robotWidth / 2;
		if (forwardOnly) {
			if (rightDisplacement > 0)
				return new double[] { 0, 2 * rightDisplacement };
			else
				return new double[] { -2 * rightDisplacement, 0 };
		}
		return new double[] { -rightDisplacement, rightDisplacement };
	}

	/**
	 * Finds the displacement of the specified {@code Pose}s (positive: forward).
	 * 
	 * @param from
	 *            the {@code Pose} before the movement
	 * @param to
	 *            the {@code Pose} after the movement
	 * @return the displacement of the specified {@code Pose}s (positive: forward)
	 */
	protected double displacement(Pose from, Pose to) {
		double distance = to.distance(from);
		if (Math.abs(Pose.normalize(from.angleTo(to) - from.directionalAngle())) < Math.PI / 2)
			return distance;
		else
			return -distance;
	}

	/**
	 * Returns the {@code Pose} of the center of the left wheels of a {@code Robot}.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a {@code Robot}
	 * @return the {@code Pose} of the center of the left wheels of a {@code Robot}
	 */
	protected Pose leftPose(Pose pose) {
		return new Pose(new Position(0, robotWidth / 2).rotate(pose.directionalAngle).translate(pose),
				pose.directionalAngle);
	}

	/**
	 * Returns the {@code Pose} of the center of the right wheels of a {@code Robot}.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a {@code Robot}
	 * @return the {@code Pose} of the center of the right wheels of a {@code Robot}
	 */
	protected Pose rightPose(Pose pose) {
		return new Pose(new Position(0, -robotWidth / 2).rotate(pose.directionalAngle).translate(pose),
				pose.directionalAngle);
	}

}
