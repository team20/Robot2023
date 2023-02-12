package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoCommands.AutoDriveDistanceCommand;
// import frc.robot.commands.AutoCommands.AutoTurnCommand;

/**
 * 
 */
public class CommandComposer {
    public static Command getDriveToTag(double distance, double angle){
        Command DriveToTag = new SequentialCommandGroup(
            // new AutoDriveDistanceCommand(distance),
            // new AutoTurnCommand(angle)
        );
        return (DriveToTag);
    }
}
