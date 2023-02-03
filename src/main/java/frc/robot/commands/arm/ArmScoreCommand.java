// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class ArmScoreCommand extends CommandBase {
	/** Creates a new ArmScore. */
	public enum ArmPosition {
		HIGH,
		MEDIUM,
		LOW
	}

	private ArmPosition m_armPosition;
	// TODO Move these values to ArmSubsytem constructor, and calculate angles once
	private double[] m_highOffsets = { 5, 9 };
	private double[] m_mediumOffsets = { 5, 0 };
	private double[] m_lowOffsets = { 0, -10 };

	public ArmScoreCommand(ArmPosition armPosition) {
		m_armPosition = armPosition;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Double[] angles = { 90.0, 0.0 };
		switch (m_armPosition) {
			case HIGH:
				angles = InverseKinematicsTool.getArmAngles(m_highOffsets[0], m_highOffsets[1]);
				break;
			case MEDIUM:
				angles = InverseKinematicsTool.getArmAngles(m_mediumOffsets[0], m_mediumOffsets[1]);
				break;
			case LOW:
				angles = InverseKinematicsTool.getArmAngles(m_lowOffsets[0], m_lowOffsets[1]);
			default:
				break;
		}
		ArmSubsystem.get().setLowerArmPosition(angles[0]);
		ArmSubsystem.get().setUpperArmPosition(angles[1]);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
