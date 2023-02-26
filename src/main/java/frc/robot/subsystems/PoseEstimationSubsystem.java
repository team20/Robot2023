// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import hlib.drive.Pose;
import hlib.drive.RobotPoseEstimator;
import hlib.drive.RobotPoseEstimatorWeighted;

/**
 * A {@code PoseEstimationSubsystem} can calculate the {@code Pose} of a
 * {@code Robot}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */

public class PoseEstimationSubsystem extends SubsystemBase {
	/**
	 * The {@code RobotPoseEstimator} used by this {@code PoseEstimationSubsystem}.
	 */
	RobotPoseEstimator poseEstimator;

	/**
	 * Construts the {@code PoseEstimationSubsystem}.
	 * 
	 * @param robotWidth
	 *                          the width of the {@code Robot}
	 * @param distanceThreshold
	 *                          the distance threshold for outlier detection (i.e.,
	 *                          the difference in x- or y-coordinates of the
	 *                          {@code Pose} from LimeLight compared to the
	 *                          {@code Pose} that has been estimated in order for
	 *                          the
	 *                          {@code Pose} from LimeLight to be considered an
	 *                          outlier)
	 * @param rejectionLimit
	 *                          the number of rejections before resetting the
	 *                          {@code PoseEstimationSubsystem}
	 * @param weight
	 *                          the weight of each new {@code Pose} from the
	 *                          LimeLight.
	 */
	public PoseEstimationSubsystem(double robotWidth, double distanceThreshold, int rejectionLimit, double weight) {
		// DriveSubsystem.get().resetEncoders();
		poseEstimator = new RobotPoseEstimatorWeighted(robotWidth, distanceThreshold, rejectionLimit, weight);
		try {
			NetworkTableInstance ins = NetworkTableInstance.getDefault(); // gets the NetworkTable
			NetworkTable table = ins.getTable("limelight"); // gets the Limelight table
			DoubleTopic v = table.getDoubleTopic("hb"); // gets the topic named hb (heartbeat)
			DoubleSubscriber s = v.subscribe(0.0); // subscribe to the topic named hb (heartbeat)
			ins.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
				// whenever hb (heartbeat) is updated
				double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]); // gets botpose
				Pose poseDetected = new Pose(botpose[0], botpose[1], botpose[5] * Math.PI / 180);
				if (poseDetected.equals(new Pose(0, 0, 0)))
					poseDetected = null;
				poseEstimator.update(poseDetected);
				SmartDashboard.putString("Pose (LimeLight)", "" + poseDetected);
			});
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
		try {
			double left = DriveSubsystem.get().getLeftEncoderPosition();
			double right = DriveSubsystem.get().getRightEncoderPosition();
			poseEstimator.update(left, right);
			// Uploads estimated pose and pose errors to the SmartDashboard.
			SmartDashboard.putString("Pose (Estimated)", "" + poseEstimator.poseEstimated());
			SmartDashboard.putString("Largest Pose Errors", "" + poseEstimator.largestPoseInconsistency());
			SmartDashboard.putString("Pose Detection Rate", (int) (1000 * poseEstimator.poseDetectionRate()) + " (ms)");
			SmartDashboard.putString("Outliers", "" + poseEstimator.outliers());
			SmartDashboard.putString("Wheel Encoder Positions", String.format("(%.3f, %.3f)", left, right));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Returns the estimated {@code Pose} of the {@code Robot}.
	 * 
	 * @return the estimated {@code Pose} of the {@code Robot}
	 */
	public Pose poseEstimated() {
		return poseEstimator.poseEstimated();
	}

}
