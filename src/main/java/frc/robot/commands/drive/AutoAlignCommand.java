// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.util.TimeThresholdedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import hlib.drive.AutoAligner;
import hlib.drive.Pose;

/**
 * A {@code AutoAlignCommand} can move the robot by a certain distance.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignCommand extends TimeThresholdedCommand {

	/**
	 * The ID of the target.
	 */
	String m_targetID;

	/**
	 * The {@code AutoAligner} used by this {@code AutoAlignCommand}.
	 */
	AutoAligner m_aligner;

	/**
	 * The {@code PoseEstimationSubsystem} used by this {@code AutoAlignCommand}.
	 */
	PoseEstimationSubsystem m_poseEstimationSubsystem;

	/**
	 * Constructs an {@code AutoAlignCommand}.
	 * 
	 * @param targetID
	 *            the ID of the target
	 * @param aligner
	 *            the {@code AutoAlginer} to use
	 * @param poseEstimationSubsystem
	 *            the {@code PoseEstimationSubsystem} to use
	 * @param timeThreshold
	 *            the time threshold (in seconds) that specifies how long the robot needs to be close to the target for
	 *            the comletion of the {@code AutoAlignCommand} (e.g., 0.1)
	 */
	public AutoAlignCommand(String targetID, AutoAligner aligner, PoseEstimationSubsystem poseEstimationSubsystem,
			double timeThreshold) {
		super(timeThreshold);
		m_targetID = targetID;
		m_aligner = aligner;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is called when this {@code AutoAlignCommand} is initially scheduled.
	 */
	@Override
	public void initialize() {
		SmartDashboard.putString("Target ID", m_targetID);
	}

	/**
	 * Is called every time the scheduler runs this {@code AutoAlignCommand}.
	 */
	@Override
	public void execute() {
		Pose poseEstimated = m_poseEstimationSubsystem.poseEstimated();
		if (poseEstimated != null) {
			double[] velocities = m_aligner.wheelVelocities(poseEstimated, m_targetID);
			if (velocities != null)
				DriveSubsystem.get().tankDrive(velocities[0], velocities[1]);
			else
				DriveSubsystem.get().tankDrive(0, 0);
		} else
			DriveSubsystem.get().tankDrive(0, 0);
	}

	/**
	 * Is called once this {@code AutoAlignCommand} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		SmartDashboard.putString("Target ID", "");
		DriveSubsystem.get().tankDrive(0, 0);
	}

	/**
	 * Determines whether or not this {@code AutoAlignCommand} meets the criteria for completion.
	 * 
	 * @return {@code true} if this {@code AutoAlignCommand} meets the criteria for completion; {@code false} otherwise
	 */
	@Override
	public boolean isFinishing() {
		Pose poseEstimated = m_poseEstimationSubsystem.poseEstimated();
		return poseEstimated != null && m_aligner.wheelVelocities(poseEstimated, m_targetID) == null;
	}
}
