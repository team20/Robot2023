package frc.robot.commands.gripperAbsolute;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AbsoluteGripperSubsystem;

public class CloseGripperCommand extends CommandBase {
    
    public CloseGripperCommand() {
        addRequirements(AbsoluteGripperSubsystem.get());
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        AbsoluteGripperSubsystem.get().setGripperMotor(0.2); // change speed as neccessary 
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if 
        
        return false;
    }
    
}



