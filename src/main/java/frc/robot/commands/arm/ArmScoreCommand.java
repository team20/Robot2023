// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class ArmScoreCommand extends CommandBase {
	Double[] angles = { 90.0, 0.0 };

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
		// ArmSubsystem.get().changeYOffset(4);
		// angles = InverseKinematicsTool.getArmAngles(ArmSubsystem.get().getXOffset(),
		// ArmSubsystem.get().getYOffset());
		// ArmSubsystem.get().setLowerArmPosition(angles[0]);
		// ArmSubsystem.get().setUpperArmPosition(angles[1]);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		switch (m_armPosition) {
			case HIGH:
				ArmSubsystem.get().setXOffset(ArmConstants.kHighOffsets[0]);
				ArmSubsystem.get().setYOffset(ArmConstants.kHighOffsets[1]);
				break;
			case MEDIUM:
				ArmSubsystem.get().setXOffset(ArmConstants.kMediumOffsets[0]);
				ArmSubsystem.get().setYOffset(ArmConstants.kMediumOffsets[1]);
				break;
			case LOW:
				ArmSubsystem.get().setXOffset(ArmConstants.kLowOffsets[0]);
				ArmSubsystem.get().setYOffset(ArmConstants.kLowOffsets[1]);
				break;
			default:
				break;
		}
		angles = InverseKinematicsTool.getArmAngles(ArmSubsystem.get().getXOffset(), ArmSubsystem.get().getYOffset());
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
