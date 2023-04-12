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
 * A {@code PoseEstimationSubsystem} can calculate the {@code Pose} of a {@code Robot}.
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
	 * Construts a {@code PoseEstimationSubsystem}.
	 * 
	 * @param robotWidth
	 *            the width of the {@code Robot}
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (i.e., the difference in x- or y-coordinates of the
	 *            {@code Pose} from the LimeLight compared to the {@code Pose} that has been estimated, in for the
	 *            {@code Pose} from the LimeLight to be considered an outlier)
	 * @param rejectionLimit
	 *            the number of rejections before resetting the {@code PoseEstimationSubsystem}
	 * @param weight
	 *            the weight of each new {@code Pose} from the LimeLight
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
				// TODO: Hwang: choose the right setting for Aster
				Pose poseDetected = new Pose(botpose[0], botpose[1], botpose[5] * Math.PI / 180 + Math.PI); // normal case - positive change : right turn
				// Pose poseDetected = new Pose(botpose[0], botpose[1], botpose[5] * Math.PI / 180); // normal case - positive change : right turn
				// Pose poseDetected = new Pose(botpose[0], botpose[1], -botpose[5] * Math.PI / 180); // negative change : right turn
				if (!isValid(poseDetected, poseEstimator))
					poseDetected = null;
				poseEstimator.update(poseDetected);
				SmartDashboard.putString("Pose (LimeLight)", "" + poseDetected);
			});
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	boolean isValid(Pose poseDetected, RobotPoseEstimator poseEstimator) {
		// TODO: Hwang: choose the right setting for Aster
		return poseDetected != null && Math.abs(poseDetected.x()) > 4.7 && Math.abs(poseDetected.x()) < 7;
		/*
		Pose poseEstimated = poseEstimator.poseEstimated();
		return poseDetected != null && Math.abs(poseDetected.x()) > 4.7 && Math.abs(poseDetected.x()) < 7
				&& (poseEstimated == null || (Math.abs(poseEstimated.x()) > 4.7 && Math.abs(poseEstimated.x()) < 7)); 
		*/
	}
	
	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
		try {
			double left = DriveSubsystem.get().getLeftEncoderPosition();
			double right = DriveSubsystem.get().getRightEncoderPosition();
			double yaw = DriveSubsystem.get().getHeading();
			// TODO choose the right setting for Aster
			poseEstimator.update(left, right, yaw * Math.PI / 180); // normal case - positive change: forward
			// poseEstimator.update(-left, -right); // negative change: forward
			// poseEstimator.update(right, left); // positive change: forward, left/right swapped
			// poseEstimator.update(-right, -left); // negative change: forward, left/right swapped

			// Uploads estimated pose and pose errors to the SmartDashboard
			SmartDashboard.putString("Pose (Estimated)", "" + poseEstimator.poseEstimated());
			SmartDashboard.putString("Largest Pose Inconsistency", "" + poseEstimator.largestPoseInconsistency());
			SmartDashboard.putString("Pose Detection Rate",
					String.format("%.2f poses / sec", poseEstimator.poseDetectionRate()));
			SmartDashboard.putString("Pose Detection Failure Rate",
					String.format("%.2f failures / sec", poseEstimator.poseDetectionFailureRate()));
			SmartDashboard.putString("Pose Outliers",
					"" + poseEstimator.outliers() + "/" + poseEstimator.posesDetected());
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
