package hlib.drive;

import java.util.Map;

/**
 * An {@code AutoAlignerBasic} can help a {@code Robot} to align to a {@code Pose}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignerBasic extends RobotPoseCalculatorBasic implements AutoAligner {

	/**
	 * The distance threshold that specifies how close the {@code Robot} needs to be within the target (e.g., 0.1).
	 */
	protected double distanceThreshold;

	/**
	 * The angle threshold that specifies how close the direction of the {@code Robot} needs to be to the direction of
	 * the target (e.g., 5 * Math.PI / 180)
	 */
	protected double angleThreshold;

	/**
	 * The maximum ratio (e.g., 0.6) that the robot can travel per tick (20ms) relative to the distance between the
	 * robot and the target.
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
	 * The target {@code Pose}s.
	 */
	protected Map<String, Pose> targetPoses;

	/**
	 * Constructs an {@code AutoAlignerBasic}.
	 * 
	 * @param robotWidth
	 *            the width of the {@code Robot}
	 * @param distanceThreshold
	 *            the distance threshold that specifies how close the {@code Robot} needs to be within the target (e.g.,
	 *            0.1)
	 * @param angleThreshold
	 *            the angle threshold that specifies how close the direction of the {@code Robot} needs to be to the
	 *            direction of the target (e.g., 5 * Math.PI / 180)
	 * @param convergenceRate
	 *            the maximum ratio (e.g., 0.6) that the robot can travel per tick (20ms) relative to the distance
	 *            between the robot and the target
	 * @param speedLimit
	 *            the maximum speed (e.g., 0.3) at which the {@code Robot} is allowed to move (1: full speed)
	 * @param maxStride
	 *            the maximum distance (e.g., 0.08) that the robot can travel per tick (20 ms)
	 * @param targetPoses
	 *            the target {@code Pose}s
	 */
	public AutoAlignerBasic(double robotWidth, double distanceThreshold, double angleThreshold, double convergenceRate,
			double speedLimit, double maxStride, Map<String, Pose> targetPoses) {
		super(robotWidth);
		this.distanceThreshold = distanceThreshold;
		this.angleThreshold = angleThreshold;
		this.convergenceRate = convergenceRate;
		this.speedLimit = speedLimit;
		this.maxStride = maxStride;
		this.targetPoses = targetPoses;
	}

	/**
	 * Returns the wheel velocities for a {@code Robot} to toward the specified target.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param targetPoseID
	 *            the ID of the target
	 * @return the wheel velocities for a {@code Robot} to move toward the specified target; {@code null} if no movement
	 *         is necessary (i.e., the {@code Robot} is close enough to the target)
	 */
	@Override
	public double[] wheelVelocities(Pose currentPose, String targetPoseID) {
		Pose p = targetPoses.get(targetPoseID);
		if (p == null)
			return new double[] { 0.0, 0.0 };
		else
			return wheelVelocities(currentPose, p);
	}

	/**
	 * Returns the wheel velocities for a {@code Robot} to toward the specified target.
	 * 
	 * @param currentPose
	 *            the current {@code Pose} of the {@code Robot}
	 * @param targetPose
	 *            the {@code Pose} of the target
	 * @return the wheel velocities for a {@code Robot} to move toward the specified target; {@code null} if no movement
	 *         is necessary (i.e., the {@code Robot} is close enough to the target)
	 */
	@Override
	public double[] wheelVelocities(Pose currentPose, Pose targetPose) {
		if (currentPose.distance(targetPose) < distanceThreshold) {
			double angleToTurn = Pose.normalize(targetPose.directionalAngle() - currentPose.directionalAngle());
			if (Math.abs(angleToTurn) < angleThreshold) // the robot is close enough to the target
				return null;
			else
				return wheelVelocitiesRotation(convergenceRate * angleToTurn);
		} else { // the robot is not close enough to the target
			double angleToTurn = Pose.normalize(currentPose.angleTo(targetPose) - currentPose.directionalAngle());
			if (Math.abs(angleToTurn) < angleThreshold)
				return wheelVelocitiesForward(convergenceRate * currentPose.distance(targetPose));
			else
				return wheelVelocitiesRotation(convergenceRate * angleToTurn);
		}
	}

	/**
	 * Returns the wheel velocities for the {@code Robot} to turn by the specified angle (in radians).
	 * 
	 * @param angle
	 *            an angle (in radians)
	 * @return the wheel velocities for the {@code Robot} to turn by the specified angle (in radians)
	 */
	protected double[] wheelVelocitiesRotation(double angle) {
		return wheelVelocities(wheelDisplacements(angle));
	}

	/**
	 * Returns the wheel velocities for the {@code Robot} to move forward/backward by the specified displacement.
	 * 
	 * @param displacement
	 *            the displacement of the movement (positive: forward, negative: backward)
	 * @return the wheel velocities for the {@code Robot} to move forward/backward by the specified displacement
	 */
	protected double[] wheelVelocitiesForward(double displacement) {
		double movement = Math.min(this.speedLimit, displacement);
		return wheelVelocities(new double[] { movement, movement });
	}

	/**
	 * Returns the wheel velocities for the {@code Robot} to move by the specified displacements of the left and right
	 * wheels.
	 * 
	 * @param displacements
	 *            the displacements of the left and right wheels of the {@code Robot} (positive: forward, negative:
	 *            backward)
	 * @return the wheel velocities for the {@code Robot} to move by the specified displacements of the left and right
	 *         wheels
	 */
	protected double[] wheelVelocities(double[] displacements) {
		double magnitude = Math.max(Math.abs(displacements[0]), Math.abs(displacements[1]));
		double scale = 1.0 / maxStride;
		if (scale * magnitude > speedLimit)
			scale = speedLimit / magnitude;
		return new double[] { scale * displacements[0], scale * displacements[1] }; // forward: wheel velocity (+)
		// return new double[] { -scale * displacements[1], -scale * displacements[0] }; // forward: wheel velocity (-), left/right swapped
	}

}
