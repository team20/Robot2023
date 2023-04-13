// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArduinoConstants;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.PixyCamObject;

public class TurnToCube extends CommandBase {
  private PixyCamObject m_cube;
  private final int m_pixyWidth = 316;
  private int m_currLocation;
  // private PIDController m_turnController = new PIDController(ArduinoConstants.kTurnP, ArduinoConstants.kTurnI, ArduinoConstants.kTurnD);

  /** Creates a new DriveToCube. */
  public TurnToCube() {
    addRequirements(DriveSubsystem.get());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_turnController.setSetpoint(0);
    m_cube = ArduinoSubsystem.get().getLargestCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currLocation = m_cube.get()[1] - 150;
    if (m_currLocation > 0){
      DriveSubsystem.get().tankDrive(0.3, -0.3);
    }
    else{
      DriveSubsystem.get().tankDrive(-0.3, 0.3);
    }
    // double turn = m_turnController.calculate(m_currLocation);
    
    System.out.println("emxcuting");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
    System.out.println("Fimished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_currLocation);
    return Math.abs(m_currLocation) < 10;
  }
}
