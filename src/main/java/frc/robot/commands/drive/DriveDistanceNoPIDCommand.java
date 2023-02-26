// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.commands.util.TimeThresholdedCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A {@code DriveDistanceNoPIDCommand} can move the robot by a certain distance.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class DriveDistanceNoPIDCommand extends TimeThresholdedCommand {

	/**
	 * The target encoder position of the left wheels.
	 */
	double m_targetPostionLeft;

	/**
	 * The target encoder postion of the right wheels.
	 */
	double m_targetPostionRight;

	/**
	 * The distance threshold that specifies how close the
	 * robot needs to be within the target (e.g., 0.1).
	 */
	double m_distanceThreshold;

	/**
	 * The maximum ratio (e.g., 0.6) that the robot can
	 * travel per tick (20ms) relative to the distance
	 * between the robot and the target.
	 */
	double m_convergenceRatio;

	/**
	 * The maximum speed (e.g., 0.3) at which the robot is
	 * allowed to move by the DriveDistanceCommand (1:
	 * full speed).
	 */
	double m_maxSpeed;

	/**
	 * The maximum distance (e.g., 0.08) that the robot can
	 * travel per tick (20 ms).
	 */
	double m_maxStride;

	/**
	 * Constructs a {@code DriveDistanceNoPIDCommand}.
	 * 
	 * @param distance
	 *                          the distance to travel (e.g., 1.0)
	 * @param distanceThreshold
	 *                          the distance threshold that specifies how close the
	 *                          robot needs to be within the target (e.g., 0.1)
	 * @param timeThreshold
	 *                          the time threshold (in ms) that specifies how long
	 *                          the robot needs to be close to the target for the
	 *                          comletion of the {@code DriveDistanceCommand2}
	 *                          (e.g., 100)
	 * @param convergenceRatio
	 *                          the maximum ratio (e.g., 0.6) that the robot can
	 *                          travel per tick (20ms) relative to the distance
	 *                          between the robot and the target
	 * @param maxSpeed
	 *                          the maximum speed (e.g., 0.3) at which the robot is
	 *                          allowed to move by the DriveDistanceCommand (1:
	 *                          full speed)
	 * @param maxStride
	 *                          the maximum distance (e.g., 0.08) that the robot can
	 *                          travel per tick (20 ms)
	 */
	public DriveDistanceNoPIDCommand(double distance, double distanceThreshold, double timeThreshold,
			double convergenceRatio, double maxSpeed, double maxStride) {
		super(timeThreshold);
		m_targetPostionLeft = DriveSubsystem.get().getLeftEncoderPosition() + distance;
		m_targetPostionRight = DriveSubsystem.get().getRightEncoderPosition() + distance;
		m_distanceThreshold = distanceThreshold;
		m_convergenceRatio = convergenceRatio;
		m_maxSpeed = maxSpeed;
		m_maxStride = maxStride;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is called when this {@code DriveDistanceNoPIDCommand} is initially scheduled.
	 */
	@Override
	public void initialize() {
	}

	/**
	 * Is called every time the scheduler runs this
	 * {@code DriveDistanceNoPIDCommand}.
	 */
	@Override
	public void execute() {
		double leftSpeed = m_convergenceRatio * (m_targetPostionLeft - DriveSubsystem.get().getLeftEncoderPosition())
				/ m_maxStride;
		double rightSpeed = m_convergenceRatio * (m_targetPostionLeft - DriveSubsystem.get().getRightEncoderPosition())
				/ m_maxStride;
		leftSpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, leftSpeed));
		rightSpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, rightSpeed));
		DriveSubsystem.get().tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * Is called once this {@code DriveDistanceNoPIDCommand} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0); // TODO set speeds
	}

	/**
	 * Determines whether or not this {@code DriveDistanceNoPIDCommand} is
	 * completed.
	 * 
	 * @return {@code true} if this {@code DriveDistanceNoPIDCommand} is completed;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinishing() {
		double leftDisplacement = DriveSubsystem.get().getLeftEncoderPosition() - m_targetPostionLeft;
		double rightDisplacement = DriveSubsystem.get().getRightEncoderPosition() - m_targetPostionRight;
		return Math.abs(leftDisplacement) < m_distanceThreshold && Math.abs(rightDisplacement) < m_distanceThreshold;
	}
}
