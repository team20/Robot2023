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
		HIGH_BACK,
		MEDIUM_FORWARD,
		MEDIUM_BACK,
		LOW,
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
	public ArmScoreCommand(ArmPosition armPosition) {
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
			case SETTLE_POSITION:
				break;
			case SUBSTATION:
				angles = ArmConstants.kSubstationAngles;
				break;
			default:
				System.out.println("IF YOU HIT THIS SOMETHING IS WRONG" + 0 / 0);
				break;
		}
		if(m_armPosition != ArmPosition.SETTLE_POSITION){
			ArmSubsystem.get().setAngles(angles[0], angles[1]);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(m_armPosition != ArmPosition.SETTLE_POSITION){
			ArmSubsystem.get().setAngles(angles[0], angles[1]);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(m_armPosition == ArmPosition.SETTLE_POSITION){
			// If the lower and upper arm is close enough to the target angle, finish the
			// command
			boolean ret = ArmSubsystem.get().isNearTargetAngle() && ranBefore;
			ranBefore = true;
			return ret;
		}

		// If the arm position is HOLD, finish the command
		if (m_armPosition == ArmPosition.HOLD) {
			return true;
		}
		// command
		boolean ret = ranBefore;
		ranBefore = true;
		return ret;
	}
}