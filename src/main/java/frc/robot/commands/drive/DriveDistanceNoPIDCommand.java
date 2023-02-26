// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A {@code DriveDistanceNoPIDCommand} can move the robot by a certain distance.
 * 
 * @author Jeong Hyon Hwang
 */
public class DriveDistanceNoPIDCommand extends CommandBase {
	double m_distance;
	double m_distanceThreshold;
	double m_timeThreshold;
	double m_maxStride;
	double m_convergenceRatio;
	double m_maxSpeed;
	Double m_convergenceStartTime = null;

	/**
	 * Constructs a {@code DriveDistanceNoPIDCommand}.
	 * 
	 * @param distance          the distance to travel (e.g., 1.0)
	 * @param distanceThreshold the distance threshold that specifies how close the
	 *                          robot needs to be within the target (e.g., 0.1)
	 * @param timeThreshold     the time threshold (in ms) that specifies how long
	 *                          the robot
	 *                          needs to be close to the target for the comletion of
	 *                          the {@code DriveDistanceCommand2} (e.g., 100)
	 * @param maxStride         the maximum distance (e.g., 0.08) that the robot can
	 *                          travel per tick (20 ms)
	 * @param convergenceRatio  the maximum ratio (e.g., 0.6) that the robot can
	 *                          travel per tick (20ms) relative to the distance
	 *                          between the robot and the target
	 * @param maxSpeed          the maximum speed (e.g., 0.3) at which the robot is
	 *                          allowed to move by the DriveDistanceCommand (1: full
	 *                          speed)
	 */
	public DriveDistanceNoPIDCommand(double distance, double distanceThreshold, double timeThreshold, double maxStride,
			double convergenceRatio, double maxSpeed) {
		m_distance = distance;
		m_distanceThreshold = distanceThreshold;
		m_timeThreshold = timeThreshold;
		m_maxStride = maxStride;
		m_convergenceRatio = convergenceRatio;
		m_maxSpeed = maxSpeed;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is called when this {@code DriveDistanceNoPIDCommand} is initially scheduled.
	 */
	@Override
	public void initialize() {
		DriveSubsystem.get().resetEncoders();
	}

	/**
	 * Is called every time the scheduler runs while this
	 * {@code DriveDistanceCommand2} is scheduled.
	 */
	@Override
	public void execute() {
		double leftSpeed = m_convergenceRatio * (m_distance - DriveSubsystem.get().getLeftEncoderPosition())
				/ m_maxStride;
		double rightSpeed = m_convergenceRatio * (m_distance - DriveSubsystem.get().getRightEncoderPosition())
				/ m_maxStride;
		leftSpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, leftSpeed));
		rightSpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, rightSpeed));
		DriveSubsystem.get().tankDrive(leftSpeed, rightSpeed);
		// System.out.println(AprilTagSubsystem.get().m_z + ", " +
		// DriveSubsystem.get().getLeftEncoderPosition() + ", "
		// + DriveSubsystem.get().getRightEncoderPosition() + ", " + leftSpeed + ", " +
		// rightSpeed);
	}

	/**
	 * Is called once this {@code DriveDistanceCommand2} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0); // TODO set speeds
	}

	/**
	 * Determines whether or not this {@code DriveDistanceNoPIDCommand} is completed.
	 * 
	 * @return {@code true} if this {@code DriveDistanceNoPIDCommand} is completed;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		double leftDisplacement = DriveSubsystem.get().getLeftEncoderPosition() - m_distance;
		double rightDisplacement = DriveSubsystem.get().getRightEncoderPosition() - m_distance;
		if (Math.abs(leftDisplacement) < m_distanceThreshold && Math.abs(rightDisplacement) < m_distanceThreshold) {
			if (m_convergenceStartTime == null)
				m_convergenceStartTime = (double) System.currentTimeMillis();
			else
				return ((double) System.currentTimeMillis() - m_convergenceStartTime) > m_timeThreshold;
		} else {
			m_convergenceStartTime = null;
		}
		return false;
	}
}
