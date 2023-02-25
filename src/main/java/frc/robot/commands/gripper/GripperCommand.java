package frc.robot.commands.gripper;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.GripperSubsystem;

//TODO revisit this command
public class GripperCommand extends CommandBase {

	//TODO remove zero
	public enum GripperPosition {
		CLOSE,
		OPEN
	}

	private GripperPosition m_gripperPosition;
	private Instant m_startTime = null;

	public GripperCommand(GripperPosition gripperPosition) {
		m_gripperPosition = gripperPosition;
		addRequirements(GripperSubsystem.get());
	}

	@Override
	public void initialize() {
		m_startTime = null;
	}

	@Override
	public void execute() {
		switch (m_gripperPosition) {
			case OPEN:
				// set gripper to open position
				GripperSubsystem.get().setGripperMotor(-GripperConstants.kMovePower); // limit switch will stop motor
				break;
			case CLOSE:
				GripperSubsystem.get().setGripperMotor(GripperConstants.kMovePower);
				if (m_startTime == null) {
					m_startTime = Instant.now();
				}
				break;
			default:
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		
	}

	@Override
	public boolean isFinished() {
		switch (m_gripperPosition) {
			case OPEN:
				return true;
			case CLOSE:
				if (Duration.between(m_startTime, Instant.now()).toMillis() >= GripperConstants.kCloseTime) {
					GripperSubsystem.get().setGripperMotor(GripperConstants.kHoldPower);
					return true;
				}
				break;
			default:
				return false;
		}
		return false;
	}
}