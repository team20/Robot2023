package frc.robot.commands.gripperAbsolute;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AbsoluteGripperSubsystem;

public class OpenGripperCommand extends CommandBase {
    
    public OpenGripperCommand() {
        addRequirements(AbsoluteGripperSubsystem.get());
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        AbsoluteGripperSubsystem.get().setGripperMotor(0);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
