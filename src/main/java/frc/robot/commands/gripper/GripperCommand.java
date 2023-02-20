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
                GripperSubsystem.get().setGripperMotor(-GripperConstants.kMovePower); // limit switch will stop motor
                break;
            case CLOSE:
                GripperSubsystem.get().setGripperMotor(GripperConstants.kMovePower);
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
    }

    @Override
    public boolean isFinished() {
        switch (m_gripperPosition) {
            case OPEN:
                m_startTime = 0;
                return true;
            case CLOSE:
                if (System.currentTimeMillis() - m_startTime >= GripperConstants.kCloseTime) {
                    GripperSubsystem.get().setGripperMotor(GripperConstants.kHoldPower);
                    m_startTime = 0;
                    return true;
                }
                break;
            default:
                return false;
        }
        return false;
    }
}