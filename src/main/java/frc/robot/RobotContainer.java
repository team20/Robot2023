// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.UpCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperWinchPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private final GenericHID m_controller = new GenericHID(ControllerConstants.kDriverControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Arm presets (high,medium,low) button triggers:
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kTriangle))
				.onTrue(new ArmScoreCommand(ArmPosition.HIGH));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(new ArmScoreCommand(ArmPosition.LOW));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kCircle))
				.onTrue(new ArmScoreCommand(ArmPosition.MEDIUM));

		// Gripper (open,close) joystick triggers:
		//Gripper opens if right joystick is pushed up
		new Trigger(() -> m_controller.getRawAxis(ControllerConstants.PS4Axis.kRightY) > ControllerConstants.kDeadzone)
				.onTrue(new GripperCommand(GripperWinchPosition.OPEN));
		//Gripper closes if right joystick is pushed down
		new Trigger(() -> m_controller.getRawAxis(ControllerConstants.PS4Axis.kRightY) < -ControllerConstants.kDeadzone)
				.onTrue(new GripperCommand(GripperWinchPosition.CLOSE));
		// new Trigger(() ->
		// m_controller.getRawButton(ControllerConstants.Button.kTriangle))
		// .onTrue(new UpCommand());

		// new Trigger(() ->
		// m_controller.getRawButton(ControllerConstants.Button.kLeftBumper))
		// .onTrue(new GripperCommand(GripperWinchPosition.OPEN));
		// new Trigger(() ->
		// m_controller.getRawButton(ControllerConstants.Button.kRightBumper))
		// .onTrue(new GripperCommand(GripperWinchPosition.CLOSE));

	}

	public Command getAutonomousCommand() {
		// return new ArmScoreCommand(ArmPosition.LOW);
		return null;
	}
}
