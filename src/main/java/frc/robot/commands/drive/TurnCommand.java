// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnCommand extends CommandBase {
  /** Creates a new TurnCommand. */
  private double m_targetAngle;


  public TurnCommand(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetAngle = targetAngle;
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currAngle = DriveSubsystem.get().getHeading();

    if(currAngle < m_targetAngle){
      DriveSubsystem.get().tankDrive(0.1, -0.1);
    }else if(currAngle < m_targetAngle){
      DriveSubsystem.get().tankDrive(-0.1, 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
