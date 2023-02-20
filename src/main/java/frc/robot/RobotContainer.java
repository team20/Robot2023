// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.arm.ChangeOffsetCommand;
import frc.robot.commands.arm.ManualMotorCommand;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.util.CommandComposer;
import frc.robot.util.ForwardKinematicsTool;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	/** The PS4 controller the operator uses */
	private final Joystick m_operatorController = new Joystick(0);

	public RobotContainer() {
		configureButtonBindings();
	}

	private boolean isArmBackwardAndButtonPressed() {
		double[] coordinates = ForwardKinematicsTool.getArmPosition(m_armSubsystem.getLowerArmAngle(),
				m_armSubsystem.getUpperArmAngle());
		return coordinates[0] < 0;
	}

	private void configureButtonBindings() {
		// Arm Controls
		m_armSubsystem.setDefaultCommand(new ChangeOffsetCommand(
				() -> m_operatorController.getRawAxis(ControllerConstants.PS4Axis.kLeftX),
				() -> m_operatorController.getRawAxis(ControllerConstants.PS4Axis.kRightY)));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.HIGH)));

		new JoystickButton(m_operatorController, ControllerConstants.Button.kSquare)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK)));

		new JoystickButton(m_operatorController, ControllerConstants.Button.kSquare)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));

		new JoystickButton(m_operatorController, ControllerConstants.Button.kX)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.LOW)));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
