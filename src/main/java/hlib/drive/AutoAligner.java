package hlib.drive;

/**
 * An {@code AutoAligner} can help a {@code Robot} to align to a {@code Pose}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface AutoAligner {

	/**
	 * Returns the wheel velocities for a {@code Robot} to move toward the specified target.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param targetPoseID
	 *            the ID of the target
	 * @return the wheel velocities for a {@code Robot} to toward the specified target; {@code null} if no movement is
	 *         necessary (i.e., the {@code Robot} is close enough to the target)
	 */
	double[] wheelVelocities(Pose currentPose, String targetPoseID);

	/**
	 * Returns the wheel velocities for a {@code Robot} to move toward the specified target.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param targetPose
	 *            the {@code Pose} of the target
	 * @return the wheel velocities for a {@code Robot} to toward the specified target; {@code null} if no movement is
	 *         necessary (i.e., the {@code Robot} is close enough to the target)
	 */
	public double[] wheelVelocities(Pose currentPose, Pose targetPose);

}
