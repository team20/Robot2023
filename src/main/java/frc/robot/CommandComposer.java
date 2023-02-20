// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.*;
//import frc.robot.commands.AutoCommands.*;
/** Add your docs here. */
//TODO: Check literally all numbers lmao
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
        //return new AutoPlacePiece(); //need parameters for how high
        //TODO: implement AutoPlacePiece
        return null; //TODO: delete after piece place is created 
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
                new BalancePIDCommand());
    }

    //Score Preloaded and Engage
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa94f33ee4_0_0
    public static Command scoreThenBalance(){ //Start lined up on center of Charge Station pushed up against nodes
        return new SequentialCommandGroup( 
                //new AutoPlacePiece(), //TODO: Still needs to be implement
                new DriveDistanceCommand(-3),
                new BalancePIDCommand());
    }

    //Score, Over Charging Station -> Out of community, backup to balance
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_8
    public static Command overTheFulcrum(){ //start lined up with middle node of coopertition zone
        return new SequentialCommandGroup(
                //new AutoPlacePiece(), //TODO: Still needs to be implement
                new DriveDistanceCommand(-7),
                new DriveDistanceCommand(3),
                new BalancePIDCommand());
    }


    //Score, Leave Community, Intake, and then Score again
    //Also can be used to Score, Over Charging Station -> Out of Community, and Score Again 
    //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1b63d19ef3d_3_0
    public static Command twoScore() {
        return new SequentialCommandGroup(
                //new AutoPlacePiece(), //TODO: need implementation 
                new TurnCommand(-180),
                new DriveDistanceCommand(7),
                //new AutoIntake(), //TODO: Make exist
                new TurnCommand(-180),
                new TagAlignCommand()//,
                /*new AutoPlacePiece()*/);//TODO: fix
    }
}