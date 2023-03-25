package hlib.drive;

/**
 * An {@code AutoAlignerSimple} can help a {@code Robot} to align to a {@code Target}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignerSimple extends RobotPoseCalculatorBasic implements AutoAligner {

	/**
	 * The maximum ratio (e.g., 0.6) that the robot can travel per tick (20ms) relative to the distance between the
	 * robot and the {@code Target}.
	 */
	protected double convergenceRate;

	/**
	 * The maximum speed (e.g., 0.3) at which the {@code Robot} is allowed to move (1: full speed).
	 */
	protected double speedLimit;

	/**
	 * The maximum distance (e.g., 0.08) that the robot can travel per tick (20 ms).
	 */
	protected double maxStride;

	/**
	 * Constructs an {@code AutoAlignerSimple}.
	 * 
	 * @param robotWidth
	 *            the width of the {@code Robot}
	 * @param convergenceRate
	 *            the maximum ratio (e.g., 0.6) that the robot can travel per tick (20ms) relative to the distance
	 *            between the robot and the target
	 * @param speedLimit
	 *            the maximum speed (e.g., 0.3) at which the {@code Robot} is allowed to move (1: full speed)
	 * @param maxStride
	 *            the maximum distance (e.g., 0.08) that the robot can travel per tick (20 ms)
	 */
	public AutoAlignerSimple(double robotWidth, double convergenceRate, double speedLimit, double maxStride) {
		super(robotWidth);
		this.convergenceRate = convergenceRate;
		this.speedLimit = speedLimit;
		this.maxStride = maxStride;
	}

	/**
	 * Determines whether or not the specified {@code Pose} is sufficiently aligned to the specified {@code Target}.
	 * 
	 * @param pose
	 *            a {@code Pose}
	 * @param target
	 *            a {@code Target}
	 * @return {@code true} if the specified {@code Pose} is sufficiently aligned to the specified {@code Target};
	 *         {@code false} otherwise
	 */	@Override
	public boolean isAligned(Pose pose, Target target) {
		return target == null || pose.distance(target) < target.distanceThreshold() && Math
				.abs(Pose.normalize(target.directionalAngle() - pose.directionalAngle())) < target.angleThreshold();
	}

	/**
	 * Returns the wheel velocities (positive: forward) for a {@code Robot} to toward the specified {@code Target}.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param target
	 *            the {@code Target}
	 * @return the wheel velocities (positive: forward) for a {@code Robot} to toward the specified {@code Target};
	 *         {@code null} if no movement is necessary (i.e., the {@code Robot} is close enough to the {@code Target})
	 */
	@Override
	public double[] wheelVelocities(Pose currentPose, Target target) {
		double displacement = currentPose.distance(target);
		double angleToTurn = Pose.normalize(currentPose.angleTo(target) - currentPose.directionalAngle());
		boolean backwardAlginment = Math.abs(angleToTurn) > Math.PI / 2;
		if (displacement < target.distanceThreshold()) { // if the target is close enough to the target
			angleToTurn = Pose.normalize(target.directionalAngle() - currentPose.directionalAngle());
			// return wheelVelocities(convergenceRate * (backwardAlginment ? -displacement : displacement),
			// convergenceRate * angleToTurn);
			return wheelVelocities(0, convergenceRate * angleToTurn);
		}
		if (backwardAlginment) { // if backward alignment
			displacement = -displacement;
			angleToTurn = Pose.normalize(angleToTurn - Math.PI);
		}
		return wheelVelocities(convergenceRate * displacement, convergenceRate * angleToTurn);
	}

	/**
	 * Returns the wheel velocities (positive: forward) for the {@code Robot} to move forward/backward by the specified
	 * displacement while turning by the specified angle.
	 * 
	 * @param displacement
	 *            the displacement of the movement (positive: forward, negative: backward)
	 * @param angle
	 *            the angle to rotate
	 * @return the wheel velocities (positive: forward) for the {@code Robot} to move forward/backward by the specified
	 *         displacement
	 */
	double[] wheelVelocities(double displacement, double angle) {
		displacement = Math.min(Math.max(displacement, -maxStride), maxStride); // enusre displacement is between -maxStride and maxStride
		Pose p = nextPose(new Pose(0, 0, angle), displacement, displacement);
		double[] displacements = wheelDisplacements(new Pose(0, 0, 0), p);
		double[] velocities = wheelVelocities(displacements);
		return velocities;
	}

	/**
	 * Returns the wheel velocities (positive: forward) for the {@code Robot} to move by the specified displacements of
	 * the left and right wheels.
	 * 
	 * @param displacements
	 *            the displacements of the left and right wheels of the {@code Robot} (positive: forward, negative:
	 *            backward)
	 * @return the wheel velocities (positive: forward) for the {@code Robot} to move by the specified displacements of
	 *         the left and right wheels
	 */
	protected double[] wheelVelocities(double[] displacements) {
		double magnitude = Math.max(Math.abs(displacements[0]), Math.abs(displacements[1]));
		double scale = 1.0 / maxStride;
		if (scale * magnitude > speedLimit)
			scale = speedLimit / magnitude;
		return new double[] { scale * displacements[0], scale * displacements[1] };
	}

}
