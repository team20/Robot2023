package frc.robot.commands.gripperAbsolute;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
    public enum GripperWinchPosition {
        OPEN,
        CLOSE
    }

    private GripperWinchPosition m_gripperWinchPosition;

    public GripperCommand(GripperWinchPosition gripperWinchPosition) {
        m_gripperWinchPosition = gripperWinchPosition;
        addRequirements(GripperSubsystem.get());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        switch (m_gripperWinchPosition) {
            case OPEN:
                while (GripperSubsystem.get().getOpenLimitSwitch() == false) {
                    // If limit switch is not pressed, motor will run
                    GripperSubsystem.get().setGripperMotor(0.1);
                }
                // If it is pressed, stop
                GripperSubsystem.get().setGripperMotor(0);
                break;
            case CLOSE:
                while (GripperSubsystem.get().getCloseLimitSwitch() == false) {
                    // If limit switch is not pressed, motor will run
                    GripperSubsystem.get().setGripperMotor(-0.1);
                }
                // If it is pressed, stop
                GripperSubsystem.get().setGripperMotor(0);
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
        return false;
    }

}
