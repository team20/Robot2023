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
import hlib.drive.Target;

/**
 * An {@code AutoAlignCommand} can move the robot to a certain {@code Target}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignCommand extends TimeThresholdedCommand {

	/**
	 * The {@code Target}.
	 */
	Target m_target;

	/**
	 * The {@code AutoAligner} used by this {@code AutoAlignCommand}.
	 */
	AutoAligner m_aligner;

	/**
	 * The {@code PoseEstimationSubsystem} used by this {@code AutoAlignCommand}.
	 */
	PoseEstimationSubsystem m_poseEstimator;

	/**
	 * The time when this {@code AutoAlignCommand} is initialized.
	 */
	long m_startTime;

	/**
	 * Constructs an {@code AutoAlignCommand}.
	 * 
	 * @param target
	 *                      the {@code Target}
	 * @param aligner
	 *                      the {@code AutoAlginer} to use
	 * @param poseEstimator
	 *                      the {@code PoseEstimationSubsystem} to use
	 */
	public AutoAlignCommand(Target target, AutoAligner aligner, PoseEstimationSubsystem poseEstimator) {
		super(0);
		m_target = target;
		m_aligner = aligner;
		m_poseEstimator = poseEstimator;
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is called when this {@code AutoAlignCommand} is initially scheduled.
	 */
	@Override
	public void initialize() {
		SmartDashboard.putString("Target", "" + m_target);
		m_startTime = System.currentTimeMillis();
		super.initialize(m_target.timeThreshold());
	}

	/**
	 * Is called every time the scheduler runs this {@code AutoAlignCommand}.
	 */
	@Override
	public void execute() {
		double[] velocities = new double[] { 0.0, 0.0 };
		Pose poseEstimated = m_poseEstimator.poseEstimated();
		if (poseEstimated != null) // know the current pose
			velocities = m_aligner.wheelVelocities(poseEstimated, m_target);
		SmartDashboard.putString("Wheel Velocities",
				String.format("(%.3f, %.3f)", velocities[0], velocities[1]));
		DriveSubsystem.get().tankDrive(velocities[0], velocities[1]); // normal case - positive: forward
		// robot.tankDrive(-velocities[0], -velocities[1]); // negative: forward
		// robot.tankDrive(velocities[1], velocities[0]); // positive: forward, left/right swapped
		// robot.tankDrive(-velocities[1], -velocities[0]); // negative: forward, left/right swapped
	}

	/**
	 * Is called once this {@code AutoAlignCommand} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		SmartDashboard.putString("Target", "");
		DriveSubsystem.get().tankDrive(0, 0);
	}

	/**
	 * Determines whether or not this {@code AutoAlignCommand} meets the criteria
	 * for completion.
	 * 
	 * @return {@code true} if this {@code AutoAlignCommand} meets the criteria for
	 *         completion; {@code false} otherwise
	 */
	@Override
	public boolean isFinishing() {
		Pose poseEstimated = m_poseEstimator.poseEstimated();
		return poseEstimated != null && m_aligner.isAligned(poseEstimated, m_target);
	}

}
