// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;
import frc.robot.commands.arm.ChangeOffsetCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.util.ForwardKinematicsTool;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final Joystick m_controller = new Joystick(ControllerConstants.kDriverControllerPort);

	public RobotContainer() {
		configureButtonBindings();
	}

	private boolean isArmBackwardAndButtonPressed() {
		double[] coordinates = ForwardKinematicsTool.getArmPosition(m_armSubsystem.getLowerArmAngle(),
				m_armSubsystem.getUpperArmAngle());
		return coordinates[0] < 0;
	}

	private void configureButtonBindings() {
		// Gripper buttons (close, open, and zero)
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kLeftBumper))
				.onTrue(new GripperCommand(GripperPosition.CLOSE));
		new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kRightBumper))
				.onTrue(new GripperCommand(GripperPosition.ZERO));
		new Trigger(() -> m_controller
				.getRawAxis(ControllerConstants.PS4Axis.kLeftTrigger) > ControllerConstants.kTriggerDeadzone)
				.onTrue(new GripperCommand(GripperPosition.OPEN));


		m_armSubsystem.setDefaultCommand(new ChangeOffsetCommand(
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kLeftX),
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kRightY)));


		// If the arm is fowards AND the button is pressed, the intermediate position
		// does not need to be used
		new Trigger(() -> !isArmBackwardAndButtonPressed()
				&& m_controller.getRawButton(ControllerConstants.Button.kTriangle))
				.onTrue(new ArmScoreCommand(ArmPosition.HIGH));

		// If we flip the arm over, go to the intermediate position, then flip the arm
		// over
		// new Trigger(
		// () -> !isArmBackwardAndButtonPressed() &&
		// m_controller.getRawButton(ControllerConstants.Button.kSquare))
		// .onTrue(new SequentialCommandGroup(new
		// ArmScoreCommand(ArmPosition.INTERMEDIATE),
		// new ArmScoreCommand(ArmPosition.MEDIUM_BACK)));

		new Trigger(
				() -> !isArmBackwardAndButtonPressed() && m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(new ArmScoreCommand(ArmPosition.MEDIUM_FORWARD));

		new Trigger(() -> !isArmBackwardAndButtonPressed() && m_controller.getRawButton(ControllerConstants.Button.kX))
				.onTrue(new ArmScoreCommand(ArmPosition.LOW));
		// If the arm is backwards AND the the button is pressed, go to the intermediate
		// position, then go to the target position
		new Trigger(() -> isArmBackwardAndButtonPressed()
				&& m_controller.getRawButton(ControllerConstants.Button.kTriangle))
				.onTrue(new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.HIGH)));

		// If the arm is backwards, we don't have to move to the intermediate position
		// before moving the arm to the back
		// new Trigger(
		// () -> isArmBackwardAndButtonPressed() &&
		// m_controller.getRawButton(ControllerConstants.Button.kSquare))
		// .onTrue(new ArmScoreCommand(ArmPosition.MEDIUM_BACK));

		new Trigger(
				() -> isArmBackwardAndButtonPressed() && m_controller.getRawButton(ControllerConstants.Button.kSquare))
				.onTrue(new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));

		new Trigger(() -> isArmBackwardAndButtonPressed() && m_controller.getRawButton(ControllerConstants.Button.kX))
				.onTrue(new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.LOW)));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}