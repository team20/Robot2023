// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TagAlignCommand;
// import frc.robot.commands.drive.TagAlignCommand;
import frc.robot.commands.drive.TurnCommand;
import frc.robot.commands.drive.TagAlignCommand.TagNumber;
import frc.robot.subsystems.AprilTagSubsystem;
// import frc.robot.commands.drive.TagAlignCommand.Position;
// import frc.robot.commands.drive.TagAlignCommand.TagNumber;
// import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
	}

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(new TagAlignCommand(TagNumber.TagGeneral, TagAlignCommand.Position.MiddlePosition, 0.75),
    new TurnCommand(3).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2),
    new TagAlignCommand(TagNumber.TagGeneral, TagAlignCommand.Position.LeftPosition, 0.75),
    new TurnCommand(3).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2),
    new TagAlignCommand(TagNumber.TagGeneral, TagAlignCommand.Position.RightPosition, 0.75),
    new TurnCommand(3).withTimeout(1),
    new WaitCommand(1),
    new DriveDistanceCommand(-2).withTimeout(2));
    // return new WaitCommand(1);
  }
}
