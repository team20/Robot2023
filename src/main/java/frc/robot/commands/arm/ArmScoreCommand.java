// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;

public class ArmScoreCommand extends CommandBase {
	double[] angles;

	public enum ArmPosition {
		HIGH,
		MEDIUM_FORWARD,
		MEDIUM_BACK,
		LOW,
		POCKET,
		INTERMEDIATE
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
				angles = ArmConstants.kHighAngles;
				break;
			case MEDIUM_FORWARD:
				angles = ArmConstants.kMediumForwardAngles;
				break;
			case MEDIUM_BACK:
				angles = ArmConstants.kMediumBackAngles;
				break;
			case LOW:
				angles = ArmConstants.kLowAngles;
				break;
			case POCKET:
				angles = ArmConstants.kPocketAngles;
				break;
			case INTERMEDIATE:
				angles = ArmConstants.kIntermediateAngles;
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
		// Calculate the arm position
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getLowerArmAngle(),
				ArmSubsystem.get().getUpperArmAngle());
		// If the y-coordinate of the upper arm is about to exceed the height, stop the
		// motors by setting their target angles to their current angles
		if (coordinates[1] > ArmConstants.kMaxHeight - 1) {
			ArmSubsystem.get().setLowerArmAngle(ArmSubsystem.get().getLowerArmAngle());
			ArmSubsystem.get().setUpperArmAngle(ArmSubsystem.get().getUpperArmAngle());
			return true;
		}
		return ArmSubsystem.get().isNearTargetAngle();
	}
}
