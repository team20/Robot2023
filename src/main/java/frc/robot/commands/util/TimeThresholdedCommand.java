// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A {@code TimeThresholdedCommand} is considered completed/finished when/after its completion criterion is met for a specified amount of time.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class TimeThresholdedCommand extends CommandBase {

	/**
	 * The time threshold (in seconds) that specifies how long the completion criterion needs to be met (e.g., 0.1).
	 */
	double m_timeThreshold;

	/**
	 * The most recent start time when the completion criterion was met.
	 */
	Long m_startTime = null;

	/**
	 * Constructs a {@code TimeThresholdedCommand}.
	 * 
	 * @param timeThreshold
	 *            the time threshold (in seconds) that specifies how long the completion criterion needs to be met (e.g., 0.1)
	 */
	public TimeThresholdedCommand(double timeThreshold) {
		m_timeThreshold = timeThreshold;
	}

    public void initialize(double timeThreshold) {
		m_timeThreshold = timeThreshold;
    }

	/**
	 * Determines whether or not this {@code TimeThresholdedCommand} is completed.
	 * 
	 * @return {@code true} if this {@code TimeThresholdedCommand} is completed; {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		if (isFinishing()) {
			if (m_startTime == null)
				m_startTime = System.currentTimeMillis();
			else
				return (System.currentTimeMillis() - m_startTime) > m_timeThreshold * 1000;
		} else
			m_startTime = null;
		return false;
	}

	/**
	 * Determines whether or not this {@code TimeThresholdedCommand} meets the criteria for completion.
	 * 
	 * @return {@code true} if this {@code TimeThresholdedCommand} meets the criteria for completion; {@code false}
	 *         otherwise
	 */
	public abstract boolean isFinishing();

}
