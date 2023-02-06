// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class ArmScoreCommand extends CommandBase {
	Double[] angles;

	public enum ArmPosition {
		HIGH,
		MEDIUM,
		LOW
	}

	private ArmPosition m_armPosition;

	/** Creates a new ArmScoreCommand. */
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
		switch (m_armPosition) {
			case HIGH:
				angles = InverseKinematicsTool.calculateArmAngles(ArmConstants.kHighOffsets[0],
						ArmConstants.kHighOffsets[1]);
				break;
			case MEDIUM:
				angles = InverseKinematicsTool.calculateArmAngles(ArmConstants.kMediumOffsets[0],
						ArmConstants.kMediumOffsets[1]);
				break;
			case LOW:
				angles = InverseKinematicsTool.calculateArmAngles(ArmConstants.kLowOffsets[0],
						ArmConstants.kLowOffsets[1]);
				break;
			default:
				break;
		}
		ArmSubsystem.get().setLowerArmAngle(angles[0]);
		ArmSubsystem.get().setUpperArmAngle(angles[1]);
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
