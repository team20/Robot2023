package hlib.drive;

/**
 * A {@code RobotPoseEstimatorBasic} can estimate the {@code Pose} of a {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class RobotPoseEstimatorBasic extends RobotPoseCalculatorBasic implements RobotPoseEstimator {

	/**
	 * The distance threshold for outlier detection (i.e., difference in x- or y-coordinates of the {@code Pose} from
	 * LimeLight compared to the {@code Pose} that has been estimated in order for the {@code Pose} from LimeLight to be
	 * considered an outlier).
	 */
	double distanceThreshold;

	/**
	 * The {@code Pose} of the {@code Robot} that has been estimated by this {@code RobotPoseEstimatorBasic}.
	 */
	Pose poseEstimated = null;

	/**
	 * The previous left wheel encoder position of the {@code Robot} (assumed to be 0 initially).
	 */
	double leftEncoderPosition = 0;

	/**
	 * The previous right wheel encoder position of the {@code Robot} (assumed to be 0 initially).
	 */
	double rightEncoderPosition = 0;

	/**
	 * The {@code PoseErrorTracker} used by this {@code RobotPoseEstimatorBasic}.
	 */
	PoseErrorTracker errorTracker = new PoseErrorTracker();

	/**
	 * The number of occasions where the {@code Pose} of the robot was detected.
	 */
	int posesDetected = 0;

	/**
	 * The number of occasions where the {@code Pose} of the robot was not detected.
	 */
	int poseDetectionFailures = 0;

	/**
	 * The number of {@code Pose} outliers that have been detected by this {@code RobotPoseEstimatorBasic}.
	 */
	int outliers = 0;

	/**
	 * The start time (in milliseconds) of this {@code RobotPoseEstimatorBasic}.
	 */
	long startTime = System.currentTimeMillis();

	/**
	 * Constructs a {@code RobotPoseEstimatorBasic}.
	 * 
	 * @param robotWidth
	 *            the width of the {@code Robot}
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (i.e., the difference in x- or y-coordinates of the
	 *            {@code Pose} from LimeLight compared to the {@code Pose} that has been estimated in order for the
	 *            {@code Pose} from LimeLight to be considered an outlier)
	 */
	public RobotPoseEstimatorBasic(double robotWidth, double distanceThreshold) {
		super(robotWidth);
		this.distanceThreshold = distanceThreshold;
	}

	/**
	 * Returns the {@code Pose} of a {@code Robot} estimated by this {@code RobotPoseEstimatorBasic}.
	 * 
	 * @return the {@code Pose} of a {@code Robot} estimated by this {@code RobotPoseEstimatorBasic}
	 */
	@Override
	public Pose poseEstimated() {
		return poseEstimated;
	}

	/**
	 * Updates this {@code RobotPoseEstimatorBasic} based on the wheel encoders of the {@code Robot}.
	 * 
	 * @param leftEncoderPosition
	 *            the left wheel encoder position of the {@code Robot}
	 * @param rightEncoderPosition
	 *            the right wheel encoder position of the {@code Robot}
	 */
	@Override
	public final synchronized void update(double leftEncoderPosition, double rightEncoderPosition) {
		double leftDisplacement = leftEncoderPosition - this.leftEncoderPosition;
		double rightDisplacement = rightEncoderPosition - this.rightEncoderPosition;
		this.leftEncoderPosition = leftEncoderPosition;
		this.rightEncoderPosition = rightEncoderPosition;
		// update(this.poseEstimated, leftDisplacement, rightDisplacement); // forward: encoder (+)
		update(this.poseEstimated, -leftDisplacement, -rightDisplacement); // forward: encoder (-)
		// update(this.poseEstimated, rightDisplacement, leftDisplacement); // forward: encoder (+), left/right swapped
	}

	/**
	 * Updates this {@code RobotPoseEstimatorBasic} based on the specified {@code Pose}.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 * @return {@code false} if the specified {@code Pose} seems like an outlier (because either the x- or the
	 *         y-coordinate value of the {@code Pose} is different by more than the threshold compared to the
	 *         {@code Pose) that has been estimated) and thus rejected; {@code true} if this {@code RobotPoseEstimator}
	 *         is updated based on the specified {@code Pose}
	 */
	@Override
	public final synchronized boolean update(Pose poseDetected) {
		if (poseDetected != null)
			posesDetected++;
		else
			poseDetectionFailures++;
		if (isOutlier(poseDetected))
			return false;
		setPoseEstimated(poseDetected);
		return true;
	}

	/**
	 * Returns the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected.
	 * 
	 * @return the rate (the number of poses per second) at which the {@code Pose} of the robot has been detected
	 */
	@Override
	public double poseDetectionRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return posesDetected / time;
		else
			return 0;
	}

	/**
	 * Returns the rate (the number of failures per second) at which the {@code Pose} of the robot has not been
	 * detected.
	 * 
	 * @return the rate (the number of failures per second) at which the {@code Pose} of the robot has not been detected
	 */
	@Override
	public double poseDetectionFailureRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return poseDetectionFailures / time;
		else
			return 0;
	}

	/**
	 * Returns the number of {@code Pose} outliers that have been detected by this {@code RobotPoseEstimator}.
	 * 
	 * @return the number of {@code Pose} outliers that have been detected by this {@code RobotPoseEstimator}
	 */
	@Override
	public int outliers() {
		return outliers;
	}

	/**
	 * Returns the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 * {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight.
	 * 
	 * @return the largest differences (in x- and y-coordinate values as well as directional angle values) between the
	 *         {@code Pose} estimated by this {@code RobotPoseEstimator} and the {@code Pose}s from the LimeLight
	 */
	@Override
	public Pose largestPoseInconsistency() {
		return this.errorTracker.largestPoseError();
	}

	/**
	 * Updates this {@code RobotPoseEstimatorBasic} based on the specified {@code Pose} and wheel displacements.
	 * 
	 * @param pose
	 *            the {@code Pose} of a {@code Robot} (estimated)
	 * @param leftDisplacement
	 *            the displacement of the left wheels of the {@code Robot}
	 * @param rightDisplacement
	 *            the displacement of the right wheels of the {@code Robot}
	 */
	protected void update(Pose pose, double leftDisplacement, double rightDisplacement) {
		this.poseEstimated = nextPose(pose, leftDisplacement, rightDisplacement);
	}

	/**
	 * Determines whether or not the specified {@code Pose} is an outlier.
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 * @return {@code true} if either the x- or the y-coordinate value of the {@code Pose} is different by more than the
	 *         threshold compared to the {@code Pose) that has been estimated; {@code false} otherwise (i.e., the
	 *         specified {@code Pose} is not an outlier)
	 */
	protected boolean isOutlier(Pose poseDetected) {
		if (poseDetected == null || this.poseEstimated == null)
			return false;
		Pose error = PoseErrorTracker.error(poseDetected, this.poseEstimated);
		if (Math.abs(error.x()) > distanceThreshold || Math.abs(error.y()) > distanceThreshold) {
			outliers++;
			// System.out.println("outlier: " + poseDetected + ", estimated " + this.poseEstimated);
			return true;
		} else
			return false;
	}

	/**
	 * Updates the {@code Pose} that has been updated by this {@code RobotPoseEstimatorBasic} based on the specified
	 * {@code Pose} from the LimeLight
	 * 
	 * @param poseDetected
	 *            the {@code Pose} from the LimeLight
	 */
	protected void setPoseEstimated(Pose poseDetected) {
		if (poseDetected != null) {
			errorTracker.update(poseDetected, this.poseEstimated);
			this.poseEstimated = poseDetected;
		}
	}

	/**
	 * Returns the number of detected {@code Pose}s that have been given to this {@code RobotPoseEstimatorBasic}.
	 * 
	 * @return the number of detected {@code Pose}s that have been given to this {@code RobotPoseEstimatorBasic}
	 */
	@Override
	public int posesDetected() {
		return this.posesDetected;
	}

}
