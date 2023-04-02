// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeCommand extends CommandBase {
  /** Creates a new DriveTime. */
  double m_speed;
  double m_time;
  Instant m_startTime = null;
  public DriveTimeCommand(double speed, double time) {
    addRequirements(DriveSubsystem.get());
    m_speed = speed;
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_startTime == null){
      m_startTime = Instant.now();
    }
    System.out.println("Running Drive Time");
    DriveSubsystem.get().tankDrive(m_speed, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Duration.between(m_startTime, Instant.now()).toMillis() > m_time;
  }
}
