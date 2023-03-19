package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.WheelGripperSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;

//TODO revisit this command
public class WheelGripperCommand extends CommandBase {

	//TODO remove zero
	public enum WheelGripperPosition {
		INTAKE,
		OUTTAKE,
		SLOW_OUTTAKE,
        STOP,
		INTAKE_CUBE_W_SENSOR
	}

	private WheelGripperPosition m_operation;

	public WheelGripperCommand(WheelGripperPosition gripperPosition) {
		m_operation = gripperPosition;
		addRequirements(WheelGripperSubsystem.get());
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		switch (m_operation) {
			case INTAKE:
                WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kMovePower);
				break;
			case OUTTAKE:
                WheelGripperSubsystem.get().setGripperMotor(GripperConstants.kMovePower);
				break;
			case SLOW_OUTTAKE:
                WheelGripperSubsystem.get().setGripperMotor(GripperConstants.kMovePower/4);
				break;
            case STOP:
                WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kHoldPower);
				break;
			case INTAKE_CUBE_W_SENSOR:
                WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kMovePower);
				break;
			default:
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		if(m_operation == WheelGripperPosition.INTAKE_CUBE_W_SENSOR){
			WheelGripperSubsystem.get().setGripperMotor(-GripperConstants.kHoldPower);
			ArduinoSubsystem.get().setCode(StatusCode.DEFAULT);
		}
	}

	@Override
	public boolean isFinished() {
		if(m_operation == WheelGripperPosition.INTAKE_CUBE_W_SENSOR){
			return WheelGripperSubsystem.get().getLimitSwitch();
		}
        return true;
	}
}