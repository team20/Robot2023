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
		// m_armSubsystem.setDefaultCommand(new
		// ArmCommand(ArmCommand.Operation.CMD_ARM_DOWN));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kTriangle))
				.onTrue(new ArmScoreCommand(ArmPosition.HIGH));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(new ArmScoreCommand(ArmPosition.LOW));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kCircle))
				.onTrue(new ArmScoreCommand(ArmPosition.MEDIUM));
		// new Trigger(() ->
		// m_controller.getRawButton(ControllerConstants.Button.kTriangle))
		// .onTrue(new UpCommand());
	}

	public Command getAutonomousCommand() {
		return new ArmScoreCommand(ArmPosition.LOW);
	}
}
