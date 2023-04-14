package hlib.drive;

/**
 * A {@code RobotPoseEstimator} can estimate the {@code Pose} of a {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface RobotPoseEstimator {

	/**
	 * Returns the {@code Pose} of a {@code Robot} estimated by this {@code RobotPoseEstimator}.
	 * 
	 * @return the {@code Pose} of a {@code Robot} estimated by this {@code RobotPoseEstimator}
	 */
	public Pose poseEstimated();

	/**
	 * Updates this {@code RobotPoseEstimator} based on the wheel encoders of the {@code Robot}.
	 * 
	 * @param leftEncoderPosition
	 *            the left wheel encoder position of the {@code Robot} (positive change: forward)
	 * @param rightEncoderPosition
	 *            the right wheel encoder position of the {@code Robot} (positive change: forward)
	 */
	public void update(double leftEncoderPosition, double rightEncoderPosition);

	/**
	 * Updates this {@code RobotPoseEstimator} based on the wheel encoders of the {@code Robot}.
	 * 
	 * @param leftEncoderPosition
	 *            the left wheel encoder position of the {@code Robot} (positive change: forward)
	 * @param rightEncoderPosition
	 *            the right wheel encoder position of the {@code Robot} (positive change: forward)
	 * @param directionalAngle
	 *            the directional angle of the {@code Robot}
	 */
	public void update(double leftEncoderPosition, double rightEncoderPosition, Double directionalAngle);

	/**
	 * Updates this {@code RobotPoseEstimator} based on the specified {@code Pose}.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} of a {@code Robot} from the LimeLight
	 * @return {@code false} if the specified {@code Pose} seems like an outlier and thus rejected; {@code true} if this
	 *         {@code RobotPoseEstimator} is updated based on the specified {@code Pose}
	 */
	public boolean update(Pose poseDetected);

	/**
	 * Returns the number of {@code Pose} outliers that have been detected by this {@code RobotPoseEstimator}.
	 * 
	 * @return the number of {@code Pose} outliers that have been detected by this {@code RobotPoseEstimator}
	 */
	public int outliers();

	/**
	 * Returns the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 * {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight.
	 * 
	 * @return the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 *         {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight
	 */
	public Pose largestPoseInconsistency();

	/**
	 * Returns the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected.
	 * 
	 * @return the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected
	 */
	double poseDetectionRate();

	/**
	 * Returns the rate (the number of failures per second) at which the {@code Pose} of the robot has not been
	 * detected.
	 * 
	 * @return the rate (the number of failures per second) at which the {@code Pose} of the robot has not been detected
	 */
	double poseDetectionFailureRate();

	/**
	 * Returns the number of detected {@code Pose}s that have been given to this {@code RobotPoseEstimator}.
	 * 
	 * @return the number of detected {@code Pose}s that have been given to this {@code RobotPoseEstimator}
	 */
	public int posesDetected();

}
