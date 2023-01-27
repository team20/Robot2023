// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.TagAlignCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return new TagAlignCommand();
  }
}
