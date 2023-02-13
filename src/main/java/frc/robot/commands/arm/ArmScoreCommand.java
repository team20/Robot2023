// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmScoreCommand extends CommandBase {
	double[] angles;

	public enum ArmPosition {
		HIGH_A,
		HIGH_B,
		MEDIUM_A,
		MEDIUM_B,
		LOW_A,
		LOW_B
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
			case HIGH_A:
				angles = ArmConstants.kHighAnglesA;
				break;
			case HIGH_B:
				angles = ArmConstants.kHighAnglesB;
			case MEDIUM_A:
				angles = ArmConstants.kMediumAnglesA;
				break;
			case MEDIUM_B:
				angles = ArmConstants.kMediumAnglesB;
			case LOW_A:
				angles = ArmConstants.kLowAnglesA;
				break;
			case LOW_B:
				angles = ArmConstants.kLowAnglesB;
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
		return ArmSubsystem.get().isNearTargetAngle();
	}
}
