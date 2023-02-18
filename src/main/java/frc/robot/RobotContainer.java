// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.UpCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private final Joystick m_controller = new Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Gripper buttons (close, open, and zero)
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kLeftBumper))
				.onTrue(new GripperCommand(GripperPosition.CLOSE));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kRightBumper))
				.onTrue(new GripperCommand(GripperPosition.ZERO));
		new Trigger(() -> m_controller.getRawAxis(ControllerConstants.PS4Axis.kLeftTrigger) > ControllerConstants.kTriggerDeadzone)
				.onTrue(new GripperCommand(GripperPosition.OPEN));
	}

	public Command getAutonomousCommand() {
		return new GripperCommand(GripperPosition.CLOSE);
		//return null;
	}
}
