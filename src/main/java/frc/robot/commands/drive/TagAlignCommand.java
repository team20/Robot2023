// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TagAlignCommand extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  PIDController m_controller = new PIDController(0.14, 0.005, 0.000); 
  //i: 0.005 before d: 0.000 before
  public TagAlignCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setSetpoint(-1);
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //DriveSubsystem.get().tankDrive(-0.25, -0.25);
    double speed = m_controller.calculate(AprilTagSubsystem.get().getDistance());
    DriveSubsystem.get().tankDrive(-speed, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
