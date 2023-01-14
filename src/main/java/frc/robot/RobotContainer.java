// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.BalanceNoPIDCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants.ControllerConstants.Axis;
public class RobotContainer {
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();

  private Joystick m_driverController = new Joystick(0);
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    DriveSubsystem.get().setDefaultCommand(new DefaultDriveCommand(
    () -> -m_driverController.getRawAxis(Axis.kLeftY),
    () -> m_driverController.getRawAxis(Axis.kLeftTrigger),
    () -> m_driverController.getRawAxis(Axis.kRightTrigger)));
  }

  public Command getAutonomousCommand() {
    return new BalanceNoPIDCommand();
  }
}
