package frc.robot.commands.gripper;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.WheelGripperSubsystem;

//TODO revisit this command
public class WheelGripperCommand extends CommandBase {

	//TODO remove zero
	public enum WheelGripperPosition {
		INTAKE,
		OUTTAKE,
        STOP
	}

	private WheelGripperPosition m_gripperPosition;

	public WheelGripperCommand(WheelGripperPosition gripperPosition) {
		m_gripperPosition = gripperPosition;
		addRequirements(WheelGripperSubsystem.get());
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		switch (m_gripperPosition) {
			case INTAKE:
                WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kMovePower);
				break;
			case OUTTAKE:
                WheelGripperSubsystem.get().setGripperMotor(GripperConstants.kMovePower);
				break;
            case STOP:
                WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kHoldPower);
			default:
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		
	}

	@Override
	public boolean isFinished() {
        return true;
	}
}