// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;

public class ArmScoreAutoCommand extends CommandBase {
	/** Stores the angles we want the arm to move to */
	double[] angles;

	/** Indicates the positions we want the arm to move to */
	public enum ArmPosition {
		HIGH,
		HIGH_BACK,
		MEDIUM_FORWARD,
		MEDIUM_BACK,
		LOW,
		LOW_AUTO,
		POCKET,
		SUBSTATION,
		TO_BACK_INTERMEDIATE,
		TO_FORWARD_INTERMEDIATE,
		HIGH_INTERMEDIATE,
		POCKET_INTERMEDIATE,
		SETTLE_POSITION,
		/** Exists to forcibly finish this command */
		HOLD
	}

	/** Stores the position we want the arm to move to */
	private ArmPosition m_armPosition;
	private boolean ranBefore;

	/** Creates a new ArmScoreCommand. */
	public ArmScoreAutoCommand(ArmPosition armPosition) {
		m_armPosition = armPosition;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Depending on the ArmPosition selected, set the angles to the corresponding
		// angle set
		switch (m_armPosition) {
			case HIGH:
				angles = ArmConstants.kHighAngles;
				break;
			case HIGH_BACK:
				angles = ArmConstants.kHighBackAngles;
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
			case LOW_AUTO:
				angles = ArmConstants.kLowAutoAngles;
				break;
			case POCKET:
				angles = ArmConstants.kPocketAngles;
				break;
			case SUBSTATION:
				angles = ArmConstants.kSubstationAngles;
				break;
			case TO_BACK_INTERMEDIATE:
				angles = ArmConstants.kToBackIntermediateAngles;
				break;
			case TO_FORWARD_INTERMEDIATE:
				angles = ArmConstants.kToFwdIntermediateAngles;
				break;
			case HIGH_INTERMEDIATE:
				angles = ArmConstants.kHighIntermediateAngles;
				break;
			case POCKET_INTERMEDIATE:
				angles = ArmConstants.kPocketIntermediateAngles;
				break;
			case HOLD:
				angles = new double[2];
				angles[0] = ArmSubsystem.get().getLowerArmAngle();
				angles[1] = ArmSubsystem.get().getUpperArmAngle();
				break;
			default:
				System.out.println("IF YOU HIT THIS SOMETHING IS WRONG" + 0 / 0);
				break;
		}
		ArmSubsystem.get().setAngles(angles[0], angles[1]);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ArmSubsystem.get().setAngles(angles[0], angles[1]);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// If the arm position is HOLD, finish the command
		if (m_armPosition == ArmPosition.HOLD) {
			return true;
		}
		// Calculate the arm position
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getLowerArmAngle(),
				ArmSubsystem.get().getUpperArmAngle());
		// If the y-coordinate of the upper arm is about to exceed the height, stop the
		// motors by setting their target angles to their current angles
		if (coordinates[1] > ArmConstants.kMaxHeight - 1) {
			ArmSubsystem.get().setAngles(ArmSubsystem.get().getLowerArmAngle(), ArmSubsystem.get().getUpperArmAngle());
			return true;
		}
		// If the lower and upper arm is close enough to the target angle, finish the
		// command
		boolean ret;
		if(m_armPosition == ArmPosition.TO_BACK_INTERMEDIATE || m_armPosition == ArmPosition.TO_FORWARD_INTERMEDIATE){
			ret = ArmSubsystem.get().isNearTargetAngleIntermediate();
		}
		ret = ArmSubsystem.get().isNearTargetAngle();
		ret = (ret && ranBefore);
		ranBefore = true;
		return ret;
	}
}