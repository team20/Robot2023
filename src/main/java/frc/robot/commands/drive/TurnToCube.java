// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.PixyCamObject;

public class TurnToCube extends CommandBase {
  private PixyCamObject m_cube;
  private final int m_pixyWidth = 316;
  private int m_currLocation;

  /** Creates a new DriveToCube. */
  public TurnToCube() {
    addRequirements(DriveSubsystem.get());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cube = ArduinoSubsystem.get().getLargestCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currLocation = m_cube.get()[1] - (m_pixyWidth /2);
    DriveSubsystem.get().tankDrive((0.2)*Math.signum(m_currLocation), (-0.2)*Math.signum(m_currLocation));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_currLocation) < 15;
  }
}
