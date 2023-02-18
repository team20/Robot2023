package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
    public enum GripperPosition {
        CLOSECUBE,
        CLOSECONE,
        OPEN
    }

    private GripperPosition m_gripperPosition;

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
                while (GripperSubsystem.get().getOpenLimitSwitch() == false) {
                    // while limit switch is not pressed, motor will run
                    GripperSubsystem.get().setGripperMotor(0.1);
                }
                // When it is pressed, stop
                GripperSubsystem.get().setGripperMotor(0);
                break;
            // To do: limit switch stuff for cube vs cone
            case CLOSECONE:
                while (GripperSubsystem.get().getCloseLimitSwitch() == false) {
                    // while limit switch is not pressed, motor will run
                    GripperSubsystem.get().setGripperMotor(0.1);
                }
                // When it is pressed, stop
                GripperSubsystem.get().setGripperMotor(0);
                break;
            case CLOSECUBE:
                while (GripperSubsystem.get().getCloseLimitSwitch() == false) {
                    // While limit switch is not pressed, motor will run
                    GripperSubsystem.get().setGripperMotor(-0.1);
                }
                // When it is pressed, stop
                GripperSubsystem.get().setGripperMotor(0);
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
        return false;
    }

}
