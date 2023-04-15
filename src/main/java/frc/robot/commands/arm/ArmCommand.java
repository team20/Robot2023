// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;

public class ArmCommand extends CommandBase {
	/** Stores the angles we want the arm to move to */
	double[] angles;

	/** Indicates the positions we want the arm to move to */
	public enum ArmPosition {
		HIGH,
		HIGH_BACK_CUBE,
		HIGH_BACK_CONE,
		MEDIUM_FORWARD,
		MEDIUM_BACK,
		LOW,
		POCKET,
		TO_BACK_INTERMEDIATE,
		TO_FORWARD_INTERMEDIATE,
		HIGH_INTERMEDIATE,
		POCKET_INTERMEDIATE,
		SUBSTATION,
		SETTLE,
		/** Exists to forcibly finish this command */
		HOLD
	}

	/** Stores the position we want the arm to move to */
	private ArmPosition m_armPosition;
	private boolean ranBefore;

	/** Creates a new ArmScoreCommand. */
	public ArmCommand(ArmPosition armPosition) {
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
				System.out.println("Setting High");
				break;
			case HIGH_BACK_CUBE:
				angles = ArmConstants.kHighBackCubeAngles;
				System.out.println("Setting Cube High Back");
				break;
			case HIGH_BACK_CONE:
				angles = ArmConstants.kHighBackConeAngles;
				System.out.println("Setting Cone High Back");
				break;
			case MEDIUM_FORWARD:
				angles = ArmConstants.kMediumForwardAngles;
				System.out.println("Setting Medium");
				break;
			case MEDIUM_BACK:
				angles = ArmConstants.kMediumBackAngles;
				System.out.println("Setting Medium Back");
				break;
			case LOW:
				angles = ArmConstants.kLowAngles;
				System.out.println("Setting Low");
				break;
			case POCKET:
				angles = ArmConstants.kPocketAngles;
				System.out.println("Setting Pocket");
				break;
			case TO_BACK_INTERMEDIATE:
				angles = ArmConstants.kToBackIntermediateAngles;
				System.out.println("Setting Back Intermidiate");
				break;
			case TO_FORWARD_INTERMEDIATE:
				angles = ArmConstants.kToFwdIntermediateAngles;
				System.out.println("Setting Forward Intermidiate");
				break;
			case HIGH_INTERMEDIATE:
				angles = ArmConstants.kHighIntermediateAngles;
				System.out.println("Setting High Intermediate");
				break;
			case POCKET_INTERMEDIATE:
				angles = ArmConstants.kPocketIntermediateAngles;
				System.out.println("Setting Pocket Intermediate");
				break;
			case SUBSTATION:
				angles = ArmConstants.kSubstationAngles;
				System.out.println("Setting Substation");
				break;
			case HOLD:
				angles = new double[2];
				angles[0] = ArmSubsystem.get().getLowerArmAngle();
				angles[1] = ArmSubsystem.get().getUpperArmAngle();
				break;
			case SETTLE:
				angles = new double[2];
				angles[0] = ArmSubsystem.get().getLowerArmSetpoint();
				angles[1] = ArmSubsystem.get().getUpperArmSetpoint();
				break;
			default:
				//System.out.println("IF YOU HIT THIS SOMETHING IS WRONG" + 0 / 0);
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

		if(m_armPosition == ArmPosition.SETTLE){
			boolean ret = ArmSubsystem.get().isNearTargetAngle();
			ret = (ret && ranBefore);
			ranBefore = true;
			return ret;
		}else{
			// If the lower and upper arm is close enough to the target angle, finish the
			// command
			boolean ret = ranBefore;
			ranBefore = true;
			return ret;
		}

		
	}
}
