// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

// import java.time.Instant;

// import java.time.Duration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveDistanceCommand extends CommandBase {
  private double m_distance;

  private ProfiledPIDController m_controller;

  /** Creates a new DriveDistanceCommand. */
  public AutoDriveDistanceCommand(double distance) {
    m_distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.get().resetEncoders();

    // Creates a ProfiledPIDController
    // Max velocity is 5 meters per second
    // Max acceleration is 10 meters per second
    
    m_controller = new ProfiledPIDController(
        DriveConstants.kP, DriveConstants.kI, DriveConstants.kD,
        new TrapezoidProfile.Constraints(125, 150)); //was 196 35 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("encoder start left", DriveSubsystem.get().getLeftEncoderPosition());
    //SmartDashboard.putNumber("encoder start right", DriveSubsystem.get().getRightEncoderPosition());

    // Calculates the output of the PID algorithm based on the sensor reading
    // and sends it to a motor
    double output = m_controller.calculate(DriveSubsystem.get().getAverageEncoderDistance(), m_distance);
    DriveSubsystem.get().tankDrive(output, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0); // TODO set speeds

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currDistanceLeft = DriveSubsystem.get().getLeftEncoderPosition();
    double currDistanceRight = DriveSubsystem.get().getRightEncoderPosition();    
    return Math.abs(DriveSubsystem.get().getAverageEncoderDistance()) > Math.abs(m_distance);
  }
}