// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.drive.*;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;

/** Add your docs here. */
public class CommandComposer {
    // Drive out of community 
    // https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.p
    public static Command outOfCommunity(double driveDistance){ //start as close to line as possible, just drive forward
        return new DriveDistanceCommand(driveDistance); //about 0.8 probably
    }

    //Balance only
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_2
    public static Command onToCharger(double driveDistance){ //start backed up a meter, figure the running start will be helpful
        return new SequentialCommandGroup(
            new DriveDistanceCommand(driveDistance),
            new BalancePIDCommand());
    }
    
    //Just Score High Game Piece
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_9
    public static Command scorePiece(){ //start backed up a meter, figure the running start will be helpful
        return getPlacePieceCommand(null); //TODO: position 
    }

    //Leave Community and balance
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_0
    public static Command leaveThenBalance(double driveDistance){ //Start right to the right of Charge station
        return new SequentialCommandGroup(
                new DriveDistanceCommand(driveDistance),
                new TurnCommand(-90), 
                new DriveDistanceCommand(2), //maybe second parameter for how far this is?
                new TurnCommand(-90), //TODO: verify all these distances
                new DriveDistanceCommand(driveDistance + 0.5), //0.5 should get on chargerstation, idk
                new BalancePIDCommand()
        );
    }

    //Score Preloaded and Engage
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa94f33ee4_0_0
    public static Command scoreThenBalance(){ //Start lined up on center of Charge Station pushed up against nodes
        return new SequentialCommandGroup( 
            getPlacePieceCommand(null), //TODO: position 
            new DriveDistanceCommand(-3),
            new BalancePIDCommand()
        );
    }

    //Score, Over Charging Station -> Out of community, backup to balance
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_8
    public static Command overTheFulcrum(){ //start lined up with middle node of coopertition zone
        return new SequentialCommandGroup(
                getPlacePieceCommand(null), //TODO: position 
                new DriveDistanceCommand(-7),
                new DriveDistanceCommand(3),
                new BalancePIDCommand()
        );
    }


    //Score, Leave Community, Intake, and then Score again
    //Also can be used to Score, Over Charging Station -> Out of Community, and Score Again 
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1b63d19ef3d_3_0
    public static Command twoScore() {
        return new SequentialCommandGroup(
                getPlacePieceCommand(null), //TODO: position 
                new TurnCommand(-180),
                new DriveDistanceCommand(7),
                getPickupPieceCommand(),
                new TurnCommand(-180),
                new TagAlignCommand(),
                getPlacePieceCommand(null)
            ); //TODO: position 
    }

    public static Command getPickupPieceCommand() {
        
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new GripperCommand(GripperPosition.OPEN),
                new ArmScoreCommand(ArmPosition.POCKET)
            ),
            new GripperCommand(GripperPosition.CLOSE)            
        );
    }

    public static Command getPlacePieceCommand(ArmPosition position) {
        return new SequentialCommandGroup(
            new ArmScoreCommand(position),
            new GripperCommand(GripperPosition.OPEN)
        );
    }
}
