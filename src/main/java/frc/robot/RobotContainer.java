// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
<<<<<<< Updated upstream
import frc.robot.commands.drive.DriveDistanceCommand2;
=======
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
=======
import frc.robot.commands.drive.DriveDistanceCommand2;
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes
import frc.robot.commands.drive.TagAlignCommand;

import frc.robot.commands.drive.TurnCommand;
import frc.robot.commands.drive.TagAlignCommand.Position;
import frc.robot.commands.drive.TagAlignCommand.TagNumber;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

/**
 * A {@code RobotContainer} contains robot subsystems, commands, and button bindings.
 */
public class RobotContainer {
  /**
   * The {@DriveSubsystem} of {@code RobotContainer}.
   */
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  /**
   * The {@ArmSubsystem} of {@code RobotContainer}.
   */
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  /**
   * The {GripperSubsystem} of {@code RobotContainer}.
   */
  private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  /**
   * The {@AprilTagSubsystem} of {@code RobotContainer}.
   */
  private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();

  /**
   * Constructs a {@code RobotContainer}.
   */
  public RobotContainer() {
    configureButtonBindings();
  }
  /**
   * Configures the buttons of the drive station's Playstation controllers
   */
  private void configureButtonBindings() {

  }
  /**
   * Returns a set of commands used at the beginning 
   * @return a set of commands used at the beginning 
   */
  public Command getAutonomousCommand() {
<<<<<<< Updated upstream
    //return new SequentialCommandGroup(new TagAlignCommand(0, -0.5));
    return new DriveDistanceCommand2(1, 0.03, 500, 0.08, 0.6, 0.1);
=======
<<<<<<< HEAD
    return new SequentialCommandGroup(new TagAlignCommand(TagNumber.TagGeneral, Position.MiddlePosition, 0.75),
    new TurnCommand(8.5).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2),
    new TagAlignCommand(TagNumber.TagGeneral, Position.LeftPosition, 0.75),
    new TurnCommand(8.5).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2),
    new TagAlignCommand(TagNumber.TagGeneral, Position.RightPosition, 0.75),
    new TurnCommand(8.5).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2));
=======
    //return new SequentialCommandGroup(new TagAlignCommand(0, -0.5));
    return new DriveDistanceCommand2(1, 0.03, 500, 0.08, 0.6, 0.1);
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes
  }
}
