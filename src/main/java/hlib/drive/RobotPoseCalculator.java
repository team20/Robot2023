package hlib.drive;

/**
 * A {@code RobotPoseCalculator} can calculate the {@code Pose} of a {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface RobotPoseCalculator {

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
	Pose nextPose(Pose pose, double leftDisplacement, double rightDisplacement);

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
	double[] wheelDisplacements(Pose from, Pose to);

	/**
	 * Returns the displacements of the left and right wheels of a {@code Robot} when it rotates by the specified angle.
	 * 
	 * @param angle
	 *            an angle (in radians)
	 * @param forwardOnly
	 *            a boolean value indicating whether or not only forward movement is allowed
	 * @return the displacements of the left and right wheels of a {@code Robot} when it rotates by the specified angle
	 */
	public double[] wheelDisplacements(double angle, boolean forwardOnly);

}
