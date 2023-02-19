// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.drive.TurnCommand;
import frc.robot.commands.arm.ChangeOffsetCommand;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;
import frc.robot.commands.arm.ChangeOffsetCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.util.CommandComposer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArduinoSubsystem;

import frc.robot.subsystems.ArduinoSubsystem.StatusCode;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	private final Joystick m_controller = new Joystick(ControllerConstants.kOperatorControllerPort);
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Gripper buttons (close, open, and zero):
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kLeftBumper))
				.onTrue(new GripperCommand(GripperPosition.CLOSE));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kRightBumper))
				.onTrue(new GripperCommand(GripperPosition.OPEN));

		// arm joysticks:
		m_armSubsystem.setDefaultCommand(new ChangeOffsetCommand(
				() -> m_controller.getRawAxis(ControllerConstants.PS4Axis.kLeftX),
				() -> m_controller.getRawAxis(ControllerConstants.PS4Axis.kRightY)));

		// arm presets:

		// If the arm is fowards, the intermediate position
		// does not need to be used to go to high
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kTriangle))
				.onTrue(CommandComposer.createArmScoreCommand(ArmPosition.HIGH));

		// If arm is forwards and target button is pressed go to intermediate position
		// and medium back
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK));

		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD));

		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kX))
				.onTrue(CommandComposer.createArmScoreCommand(ArmPosition.LOW));

		// LED cube and cone
		new Trigger(() -> m_controller.getPOV() == ControllerConstants.DPad.kLeft)
				.onTrue(new LEDCommand(StatusCode.PURPLE_BLINKING));
		new Trigger(() -> m_controller.getPOV() == ControllerConstants.DPad.kRight)
				.onTrue(new LEDCommand(StatusCode.YELLOW_BLINKING));

		// ------------driver controls------------------

		// gripper: drop/open
		new Trigger(() -> m_driverController.getRawButton(ControllerConstants.Button.kX))
				.onTrue(new GripperCommand(GripperPosition.OPEN));
		// last year's code for drive: left joystick and left/right triggers

		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem,
						() -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> m_driverController.getRawAxis(Axis.kLeftTrigger),
						() -> m_driverController.getRawAxis(Axis.kRightTrigger)));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
