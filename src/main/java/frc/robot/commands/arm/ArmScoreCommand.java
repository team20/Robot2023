// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;

public class ArmScoreCommand extends CommandBase {
	/** Stores the angles we want the arm to move to */
	double[] angles;

	/** Indicates the positions we want the arm to move to */
	public enum ArmPosition {
		HIGH,
		MEDIUM_FORWARD,
		MEDIUM_BACK,
		LOW,
		POCKET,
		INTERMEDIATE
	}

	/** Stores the position we want the arm to move to */
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
		// Depending on the ArmPosition selected, set the angles to the corresponding
		// angle set
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
		ArmSubsystem.get().setAngles(angles[0], angles[1]);
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
			// TODO remove commented print line
			// System.out.println("Height limit protection tripped!");
			ArmSubsystem.get().setAngles(ArmSubsystem.get().getLowerArmAngle(), ArmSubsystem.get().getUpperArmAngle());
			return true;
		}
		// If the lower and upper arm is close enough to the target angle, finish the
		// command
		return ArmSubsystem.get().isNearTargetAngle();
	}
}
