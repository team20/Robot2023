package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
	public enum GripperPosition {
		CLOSE,
		OPEN,
		ZERO
	}

	private GripperPosition m_gripperPosition;
	private long m_startTime = 0;

	public GripperCommand(GripperPosition gripperPosition) {
		m_gripperPosition = gripperPosition;
		addRequirements(GripperSubsystem.get());
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		switch (m_gripperPosition) {
			case OPEN:
				// set gripper to open position
				GripperSubsystem.get().setGripperPosition(GripperConstants.kGripperOpenPosition);
				break;
			case CLOSE:
				GripperSubsystem.get().setGripperMotor(.1);
				if (m_startTime == 0) {
					m_startTime = System.currentTimeMillis();
				}
				break;
			case ZERO:
				GripperSubsystem.get().setGripperMotor(.1);
				if (m_startTime == 0) {
					m_startTime = System.currentTimeMillis();
				}
				break;
			default:
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		GripperSubsystem.get().setGripperMotor(0);
	}

	@Override
	public boolean isFinished() {
		switch (m_gripperPosition) {
			case OPEN:
				if (Math.abs(GripperSubsystem.get().getGripperEncoderPosition()
						- GripperConstants.kGripperOpenPosition) < 10) {
					GripperSubsystem.get().setGripperMotor(0);
					return true;
				}
				break;
			case CLOSE:
				if (System.currentTimeMillis() - m_startTime >= GripperConstants.kCloseTime) {
					GripperSubsystem.get().setGripperMotor(GripperConstants.kHoldPower);
					return true;
				}
				break;
			case ZERO:
				if (System.currentTimeMillis() - m_startTime > GripperConstants.kCloseTime) {
					GripperSubsystem.get().setGripperMotor(0);
					GripperSubsystem.get().resetZero();
					return true;
				}
				break;
			default:
				return false;
		}
		return false;
	}

}
