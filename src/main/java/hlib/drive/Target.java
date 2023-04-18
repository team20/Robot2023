package hlib.drive;

/**
 * A {@code Target} represents the pose of a target in a 2-dimensional space.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class Target extends Pose {

	/**
	 * The distance threshold that specifies how close the {@code Robot} needs to be to the target (e.g., 0.1).
	 */
	double distanceThreshold;

	/**
	 * The angle threshold that specifies how close the direction of the {@code Robot} needs to be to the direction of
	 * the target (e.g., 5 * Math.PI / 180).
	 */
	double angleThreshold;

	/**
	 * The time threshold (in seconds) that specifies how long the robot needs to be close to the target for the
	 * completion of the alignment (e.g., 0.1).
	 */
	double timeThreshold;

	/**
	 * Constructs a {@code Target}.
	 * 
	 * @param x
	 *            the x-coordinate value of the {@code Target}
	 * @param y
	 *            the y-coordinate value of the {@code Target}
	 * @param directionalAngle
	 *            the directional angle (in radians) of the {@code Target}
	 * @param distanceThreshold
	 *            the distance threshold that specifies how close the {@code Robot} needs to be to the target (e.g.,
	 *            0.1)
	 * @param angleThreshold
	 *            the angle threshold that specifies how close the direction of the {@code Robot} needs to be to the
	 *            direction of the target (e.g., 5 * Math.PI / 180)
	 * @param timeThreshold
	 *            the time threshold (in seconds) that specifies how long the robot needs to be close to the target for
	 *            the completion of the alignment (e.g., 0.1)
	 */
	public Target(double x, double y, double directionalAngle, double distanceThreshold, double angleThreshold,
			double timeThreshold) {
		super(x, y, directionalAngle);
		this.distanceThreshold = distanceThreshold;
		this.angleThreshold = angleThreshold;
		this.timeThreshold = timeThreshold;
	}

	/**
	 * Returns the distance threshold that specifies how close the {@code Robot} needs to be to the target (e.g.,
	 * 0.1).
	 * 
	 * @return the distance threshold that specifies how close the {@code Robot} needs to be to the target (e.g.,
	 *         0.1)
	 */
	public double distanceThreshold() {
		return distanceThreshold;
	}

	/**
	 * Returns the angle threshold that specifies how close the direction of the {@code Robot} needs to be to the
	 * direction of the target (e.g., 5 * Math.PI / 180).
	 * 
	 * @return the angle threshold that specifies how close the direction of the {@code Robot} needs to be to the
	 *         direction of the target (e.g., 5 * Math.PI / 180)
	 */
	public double angleThreshold() {
		return angleThreshold;
	}

	/**
	 * Returns the time threshold (in seconds) that specifies how long the robot needs to be close to the target for the
	 * completion of the alignment (e.g., 0.1).
	 * 
	 * @return the time threshold (in seconds) that specifies how long the robot needs to be close to the target for the
	 *         completion of the alignment (e.g., 0.1)
	 */
	public double timeThreshold() {
		return timeThreshold;
	}

	/**
	 * Determines whether or not the given {@code Object} is equal to this {@code Target}.
	 * 
	 * @return {@code true} if the given {@code Object} is equal to this {@code Target}; {@code false} otherwise
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Target) {
			Target p = (Target) o;
			return super.equals(p) && this.distanceThreshold == p.distanceThreshold
					&& this.angleThreshold == p.angleThreshold && this.timeThreshold == p.timeThreshold;
		} else
			return false;
	}

	/**
	 * Returns a {@code String} representation of this {@code Target}.
	 * 
	 * @return a {@code String} representation of this {@code Target}
	 */
	@Override
	public String toString() {
		return String.format("(%.3f, %.3f, %.1f degrees, %.2f meters, %.1f degrees, %.2f seconds)", x, y,
				directionalAngle * 180 / Math.PI, distanceThreshold, angleThreshold * 180 / Math.PI, timeThreshold);
	}

}
