// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PoseEstimationSubsystem;
import hlib.drive.AutoAligner;
import hlib.frc.TargetMap;

/**
 * A {@code AutoAlignCommandConfigurable} can move the robot to a certain {@code Target}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
/**
 * A {@code AutoAlignCommandConfigurable} can move the robot to a certain {@code Target} specified by a topic in the {@code SmartDashboard}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignCommandConfigurable extends AutoAlignCommand {

	/**
	 * The name of the topic in the {@code SmartDashboard}.
	 */
	String m_topicName;

	/**
	 * The ID of the {@code Target}.
	 */
	String m_targetID;

	/**
	 * The {@code TargetMap} used by this {@code AutoAlignCommandConfigurable}.
	 */
	TargetMap m_targetMap;

	/**
	 * Constructs an {@code AutoAlignCommandConfigurable}.
	 * 
	 * @param topicName
	 *            the name of the topic in the {@code SmartDashboard}
	 * @param aligner
	 *            the {@code AutoAlginer} to use
	 * @param poseEstimator
	 *            the {@code PoseEstimationSubsystem} to use
	 * @param targetMap
	 *            a {@code TargetMap}
	 */
	public AutoAlignCommandConfigurable(String topicName, AutoAligner aligner, PoseEstimationSubsystem poseEstimator,
			TargetMap targetMap) {
		super(null, aligner, poseEstimator);
		m_topicName = topicName;
		this.m_targetMap = targetMap;
	}

    /**
	 * Is called when this {@code AutoAlignCommandConfigurable} is initially scheduled.
	 */
	@Override
	public void initialize() {
		m_targetID = SmartDashboard.getString(m_topicName, "");
		SmartDashboard.putString("Target ID", m_targetID);
		m_target = m_targetMap.get(m_targetID);
		super.initialize();
	}

	/**
	 * Is called once this {@code AutoAlignCommandConfigurable} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		SmartDashboard.putString("Target ID", "");
		if (!interrupted)
		SmartDashboard.putString("Last Travel Time", String.format("%.2f seconds to target %s",
					0.001 * (System.currentTimeMillis() - m_startTime), m_targetID));
		super.end(interrupted);
	}

}