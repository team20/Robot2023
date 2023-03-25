package hlib.drive;

/**
 * An {@code AutoAligner} can help a {@code Robot} to align to a {@code Target}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface AutoAligner {

	/**
	 * Returns the wheel velocities (positive: forward) for a {@code Robot} to move toward the specified {@code Target}.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param target
	 *            a {@code Target}
	 * @return the wheel velocities (positive: forward) for a {@code Robot} to toward the specified {@code Target};
	 *         {@code null} if no movement is necessary (i.e., the {@code Robot} is close enough to the {@code Target})
	 */
	public double[] wheelVelocities(Pose currentPose, Target target);

	/**
	 * Determines whether or not the specified {@code Pose} is sufficiently aligned to the specified {@code Target}.
	 * 
	 * @param pose
	 *            a {@code Pose}
	 * @param target
	 *            a {@code Target}
	 * @return {@code true} if the specified {@code Pose} is sufficiently aligned to the specified {@code Target};
	 *         {@code false} otherwise
	 */
	boolean isAligned(Pose pose, Target target);

}
