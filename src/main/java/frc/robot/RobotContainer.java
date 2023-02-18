// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArduinoConstants;
import frc.robot.subsystems.ArduinoSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.UpCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final GenericHID m_controller = new GenericHID(ControllerConstants.kDriverControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
