// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.arm.ManualMotorCommand;
import frc.robot.commands.drive.BalancePIDCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveBrakeModeCommand;
import frc.robot.commands.drive.DriveToApriltag;
import frc.robot.commands.gripper.WheelGripperCommand;
import frc.robot.commands.gripper.WheelGripperCommand.WheelGripperPosition;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.commands.util.DeferredCommandAuto;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WheelGripperSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.util.CommandComposer;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private WheelGripperSubsystem m_gripperSubsystem = new WheelGripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	/** The PS4 controller the operator uses */
	private final Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);
	/** The PS4 controller the driver uses */
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);

	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

	public RobotContainer() {
		// m_autoChooser.addOption("Out of Community", CommandComposer.getOutOfCommunityAuto(0));
		// m_autoChooser.addOption("Onto Charge Station", CommandComposer.getOnToChargerAuto(0));
		// m_autoChooser.addOption("Score 1 piece", CommandComposer.getScorePieceAuto());
		// m_autoChooser.addOption("Leave then balance", CommandComposer.getLeaveThenBalanceAuto(1));// TODO fix distance
		m_autoChooser.addOption("Score then balance", CommandComposer.getScoreThenBalanceAuto());
		m_autoChooser.addOption("Over Charge Station", CommandComposer.getOverTheFulcrumAuto());
		m_autoChooser.addOption("Over Charge Station NO SCORE", CommandComposer.getOverTheFulcrumNoScoreAuto());
		m_autoChooser.addOption("Score then leave", CommandComposer.getScoreThenLeaveCommand());
		m_autoChooser.addOption("Just leave", CommandComposer.getJustLeaveCommand());
		m_autoChooser.addOption("Score two RED", CommandComposer.getTwoScoreRedAuto());
		m_autoChooser.addOption("Score two BLUE", CommandComposer.getTwoScoreBlueAuto());
		// m_autoChooser.addOption("Score two and balance", CommandComposer.getTwoScoreBalanceAuto());
		SmartDashboard.putData(m_autoChooser);
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// -------------Gripper Controls-------------
		new JoystickButton(m_operatorController, ControllerConstants.Button.kLeftBumper)
			.and(() -> !m_operatorController.getRawButton(Button.kLeftTrigger))
			.onTrue(new SequentialCommandGroup(new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR)));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kLeftBumper)
						.and(() -> m_operatorController.getRawButton(Button.kLeftTrigger))
						.onTrue(new SequentialCommandGroup(new WheelGripperCommand(WheelGripperPosition.INTAKE),
								(new LEDCommand(StatusCode.DEFAULT))));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kRightBumper)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.STOP));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kRightTrigger)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kOptions)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.SLOW_OUTTAKE));

		// -------------Arm Controls-------------
		m_armSubsystem.setDefaultCommand(new ManualMotorCommand(
				() -> m_operatorController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_operatorController.getRawAxis(ControllerConstants.Axis.kRightY)));

		// Move the arm to the high node
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle).and(() -> !m_operatorController.getRawButton(Button.kLeftTrigger))
				.onTrue(new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.HIGH_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.SETTLE_POSITION),
						new ArmScoreCommand(ArmPosition.HIGH)));

		// Move the arm to the medium node over the back
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.and(() -> m_operatorController.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.HIGH_BACK)));

		// Move the arm to the medium node
		new JoystickButton(m_operatorController, ControllerConstants.Button.kSquare)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));

		// Move the arm to the medium node over the back
		new JoystickButton(m_operatorController, ControllerConstants.Button.kCircle)
				.and(() -> m_operatorController.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK)));

		// Move the arm to the low position
		new JoystickButton(m_operatorController, ControllerConstants.Button.kX)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.LOW)));

		// Move the arm to the pocket
		new JoystickButton(m_operatorController, ControllerConstants.Button.kCircle).and(() -> !m_operatorController.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.POCKET)));

		new JoystickButton(m_operatorController, ControllerConstants.Button.kTrackpad)
				.onTrue(new ArmScoreCommand(ArmPosition.HOLD));

    // Move the arm to the substation position
	new JoystickButton(m_operatorController, ControllerConstants.Button.kSquare).and(() -> m_operatorController.getRawButton(Button.kLeftTrigger))
    .onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.SUBSTATION)));


		// -------------LED signaling-------------
		// // Signal for a cube
		new POVButton(m_operatorController, ControllerConstants.DPad.kLeft)
				.onTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));
		// Signal for a cone
		new POVButton(m_operatorController, ControllerConstants.DPad.kRight)
				.onTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kRightBumper)
				.onTrue(new LEDCommand(StatusCode.DEFAULT));
		// new POVButton(m_operatorController, ControllerConstants.DPad.kDown)
		// 		.onTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));

		// -------------Driver Controls-------------
		// Opening gripper/dropping game piece
		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.whileTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));
		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.whileTrue(new WheelGripperCommand(WheelGripperPosition.SLOW_OUTTAKE));

		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle).whileTrue(new DriveBrakeModeCommand());
		// Driving
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger)));

		// Fine Turning
		new JoystickButton(m_driverController, ControllerConstants.Button.kRightBumper)
			.whileTrue(new DefaultDriveCommand(()->0.0, ()->0.0, ()->DriveConstants.kFineTurningSpeed));
		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
			.whileTrue(new DefaultDriveCommand(()->0.0, ()->DriveConstants.kFineTurningSpeed, ()->0.0));
	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}
}
