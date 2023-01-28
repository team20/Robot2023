// package frc.robot.commands.AutoCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Translation2d;
// // import edu.wpi.first.math.trajectory.Trajectory;
// // import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// // import edu.wpi.first.wpilibj2.command.CommandBase;
// // import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.CommandComposer;
// import frc.robot.commands.drive.DriveDistanceCommand;
// // import frc.robot.Constants.DriveConstants;
// // import frc.robot.Constants.FieldLocation;
// // import frc.robot.commands.ShooterCommands.AutoIndexCommand;
// // import frc.robot.commands.ShooterCommands.ShootSetupCommand;
// import frc.robot.subsystems.DriveSubsystem;

// public class AutoSequence extends SequentialCommandGroup {


//     public AutoSequence(DriveSubsystem driveSubsystem, int choice) {
      
//         // TODO integrate into the CommandComposer 
      
//         //Not sure if the distance is in meters, it didn't function as such when tested, PIDs need to be tuned?
//         switch(choice) {
//             // Drive out of community 
//             // https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.p
//             case 1: //start as close to line as possible, just drive forward
//                 addCommands(new AutoDriveDistanceCommand(0.8)); //about 0.8 probably
//                 break;

//             //Balance only
//             //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_2
//             case 2: //start backed up a meter, figure the running start will be helpful
//                 addCommands(new AutoDriveDistanceCommand(1), 
//                 new AutoBalanceCommand());
//                 break;

//             //Just Score High Game Piece
//             //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_9
//             case 3: //start lined up, probably on middle one to work towards coop links quick, maybe on side to allow more access for someone else to charging station
//                 addCommands(new AutoPlacePiece()); //need parameters for which piece and how high
//                 break;

//             //Leave Community and balance
//             //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_0
//             case 4: //Start right to the left of Charge station
//                 addCommands(new AutoDriveDistanceCommand(2),
//                 new AutoTurnCommand(-90), 
//                 new AutoDriveDistanceCommand(2),
//                 new AutoTurnCommand(-90),
//                 new DriveDistanceCommand(2.5),
//                 new AutoBalanceCommand());
//                 break;

//             //Score Preloaded and Engage
//             //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa94f33ee4_0_0
//             case 5: //Start lined up on center of Charge Station pushed up against nodes
//                 addCommands(new AutoPlacePiece(), 
//                 new AutoDriveDistanceCommand(-3),
//                 new AutoBalanceCommand());
//                 break;

//             //Score, Over Charging Station -> Out of community, backup to balance
//             //https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_8
//             case 6: //start lined up with middle node of coopertition zone
//                 addCommands(new AutoPlacePiece(),
//                 new AutoDriveDistanceCommand(-7),
//                 new AutoDriveDistanceCommand(3),
//                 new AutoBalanceCommand());
//                 break;
            
//             //Not a real one yet, to test Apriltag stuff
//             // case 7:
//             // addCommands(CommandComposer.getDriveToTag());//get distance and angle from newtwork tables
//         }
//     }

// }