// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.arm.ManualMotorCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
	// private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	// private WheelGripperSubsystem m_gripperSubsystem = new
	// WheelGripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	// private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	// TODO: Hwang: check the scaling constant

	/** The PS4 controller the operator uses */
	// private final Joystick m_operatorController1 = new Joystick(0);
	private final CommandPS4Controller m_operatorController = new CommandPS4Controller(
			ControllerConstants.kOperatorControllerPort);
	/** The PS4 controller the driver uses */
	// private final Joystick m_driverController = new
	// Joystick(ControllerConstants.kDriverControllerPort);

	private final CommandPS4Controller m_driverController1 = new CommandPS4Controller(0);
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

	public RobotContainer() {
		// m_autoChooser.addOption("Out of Community",
		// CommandComposer.getOutOfCommunityAuto(0));
		// m_autoChooser.addOption("Onto Charge Station",
		// CommandComposer.getOnToChargerAuto(0));
		// m_autoChooser.addOption("Score 1 piece",
		// CommandComposer.getScorePieceAuto());
		// m_autoChooser.addOption("Leave then balance",
		// CommandComposer.getLeaveThenBalanceAuto(1));// TODO fix distance
		// m_autoChooser.addOption("Score two and balance",
		// CommandComposer.getTwoScoreBalanceAuto());
		SmartDashboard.putData(m_autoChooser);
		SmartDashboard.putData("gwjrijgeni", m_autoChooser);
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		m_armSubsystem.setDefaultCommand(
				new ManualMotorCommand(() -> -m_driverController1.getLeftY(), () -> -m_driverController1.getRightY()));
		// -------------LED signaling-------------
		// Signal for a cube
		new POVButton(m_driverController1.getHID(), ControllerConstants.DPad.kLeft)
				.onTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));

		// // Signal for a cone
		new POVButton(m_driverController1.getHID(), ControllerConstants.DPad.kRight)
				.onTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		new POVButton(m_driverController1.getHID(), ControllerConstants.DPad.kUp)
				.onTrue(new LEDCommand(StatusCode.RAINBOW_PARTY_FUN_TIME));
		new JoystickButton(m_driverController1.getHID(),
				ControllerConstants.Button.kRightBumper)
						.onTrue(new LEDCommand(StatusCode.DEFAULT));
		new POVButton(m_driverController1.getHID(), ControllerConstants.DPad.kDown)
				.onTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));
	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		// return new BalancePIDCommand();
		return m_autoChooser.getSelected();
	}
}
