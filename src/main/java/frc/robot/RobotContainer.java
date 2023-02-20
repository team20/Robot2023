// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.gripper.*;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;
import frc.robot.commands.util.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.util.*;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	/** The PS4 controller the operator uses */
	private final Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);
	/** The PS4 controller the driver uses */
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);

	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
	public RobotContainer() {
		m_autoChooser.addOption("Out of Community", CommandComposer.getOutOfCommunityAuto(0));
		m_autoChooser.addOption("Onto Charge Station", CommandComposer.getOnToChargerAuto(0));
		m_autoChooser.addOption("Score 1 piece", CommandComposer.getScorePieceAuto());
		m_autoChooser.addOption("Leave then balance", CommandComposer.getLeaveThenBalanceAuto(1));//TODO fix distance
		m_autoChooser.addOption("Score then balance", CommandComposer.getScoreThenBalanceAuto());
		m_autoChooser.addOption("Score, leave over charge, balance", CommandComposer.getOverTheFulcrumAuto());
		m_autoChooser.addOption("Score two", CommandComposer.getTwoScoreAuto());
		m_autoChooser.addOption("Score two and balance", CommandComposer.getTwoScoreBalanceAuto());
		SmartDashboard.putData(m_autoChooser);
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// Gripper buttons (close, open, and zero):
		new JoystickButton(m_operatorController, ControllerConstants.Button.kLeftBumper)
				.whileTrue(new GripperCommand(GripperPosition.CLOSE));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kRightBumper)
				.whileTrue(new GripperCommand(GripperPosition.OPEN));

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
		// LED cube and cone

		new POVButton(m_operatorController, ControllerConstants.DPad.kLeft)
				.whileTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));
		new POVButton(m_operatorController, ControllerConstants.DPad.kRight)
				.whileTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		new POVButton(m_operatorController, ControllerConstants.DPad.kUp)
				.whileTrue(new LEDCommand(StatusCode.DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR));
		new POVButton(m_operatorController, ControllerConstants.DPad.kDown)
				.whileTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));

		new JoystickButton(m_operatorController, ControllerConstants.Button.kLeftBumper)
				.whileTrue(new LEDCommand(StatusCode.DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR));

		// -------------Driver Controls-------------
		// Opening gripper/dropping game piece
		new JoystickButton(m_operatorController, ControllerConstants.Button.kX)
				.whileTrue(new GripperCommand(GripperPosition.OPEN));
		// Driving
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.PS4Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.PS4Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.PS4Axis.kRightTrigger)));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
