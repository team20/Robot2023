// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.arm.ManualMotorCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveBrakeModeCommand;
import frc.robot.commands.drive.TurnTimeCommand;
import frc.robot.commands.gripper.WheelGripperCommand;
import frc.robot.commands.gripper.WheelGripperCommand.WheelGripperPosition;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.WheelGripperSubsystem;
import frc.robot.util.CommandComposer;

public class RobotContainer {
	private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private WheelGripperSubsystem m_gripperSubsystem = new WheelGripperSubsystem();
	private ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
	// TODO: Hwang: check the scaling constant
	PoseEstimationSubsystem m_poseSubsystem = new PoseEstimationSubsystem(DriveConstants.kTrackwidthMeters * 300 / 360,
			0.5,
			10, 0.1);

	/** The PS4 controller the operator uses */
	private final Joystick m_operatorController1 = new Joystick(ControllerConstants.kOperatorControllerPort);
	private final CommandPS4Controller m_operatorController = new CommandPS4Controller(
			ControllerConstants.kOperatorControllerPort);
	/** The PS4 controller the driver uses */
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);

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
		m_autoChooser.addOption("Score then balance", CommandComposer.getScoreThenBalanceAuto());
		m_autoChooser.addOption("Over Charge Station Backwards", CommandComposer.getOverTheFulcrumAuto());
		m_autoChooser.addOption("Over Charge Station Forwards", CommandComposer.getOverTheFulcrumForwardAuto());
		m_autoChooser.addOption("Over Charge Station NO SCORE", CommandComposer.getOverTheFulcrumNoScoreAuto());
		m_autoChooser.addOption("Score then leave", CommandComposer.getScoreThenLeaveCommand());
		m_autoChooser.addOption("Just leave", CommandComposer.getJustLeaveCommand());
		m_autoChooser.addOption("Score two RED", CommandComposer.getTwoScoreRedAuto());
		m_autoChooser.addOption("Score two BLUE", CommandComposer.getTwoScoreBlueAuto());
		m_autoChooser.addOption("Score two RED Wire Bump", CommandComposer.getTwoScoreRedAuto());
		m_autoChooser.addOption("Score two BLUE Wire Bump", CommandComposer.getTwoScoreBlueWireBumpAuto());
		// m_autoChooser.addOption("Score two and balance",
		// CommandComposer.getTwoScoreBalanceAuto());
		SmartDashboard.putData(m_autoChooser);
		configureButtonBindings();
	}

	private void configureButtonBindings2() {
		// Move the arm to the high node
		m_operatorController.triangle().and(m_operatorController.L2().negate())
				.onTrue(new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.HIGH_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.HIGH)));

		// Move the arm to the high node over the back
		m_operatorController.triangle().and(m_operatorController.L2())
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.HIGH_BACK)));

		// Move the arm to the medium node
		m_operatorController.square()
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));

		// Move the arm to the medium node over the back
		m_operatorController.circle().and(m_operatorController.L2())
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK)));

		// Move the arm to the low position
		m_operatorController.cross()
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.LOW)));

		// Move the arm to the pocket
		m_operatorController.circle().and(m_operatorController.L2().negate())
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.POCKET)));

		m_operatorController.touchpad().onTrue(new ArmScoreCommand(ArmPosition.HOLD));

		// Move the arm to the substation position
		m_operatorController.square().and(m_operatorController.L2())
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.SUBSTATION)));

		// -------------LED signaling-------------
		// Signal for a cube
		m_operatorController.povLeft().onTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));

		// Signal for a cone
		m_operatorController.povRight().onTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		m_operatorController.povUp().onTrue(new LEDCommand(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_operatorController.R2().onTrue(new LEDCommand(StatusCode.DEFAULT));
		// new POVButton(m_operatorController, ControllerConstants.DPad.kDown)
		// .onTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));

		// -------------Driver Controls-------------
		// Opening gripper/dropping game piece
		m_driverController1.cross().whileTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));
	}

	private void configureButtonBindings() {
		// -------------Gripper Controls-------------
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kLeftTrigger)
				.and(() -> !m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new SequentialCommandGroup(new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR)));
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kLeftBumper)
				.and(() -> m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new SequentialCommandGroup(new WheelGripperCommand(WheelGripperPosition.INTAKE),
						(new LEDCommand(StatusCode.DEFAULT))));
		new JoystickButton(m_operatorController1, Button.kRightBumper)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.STOP));
		new JoystickButton(m_operatorController1, Button.kRightTrigger)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));
		new JoystickButton(m_operatorController1, Button.kOptions)
				.onTrue(new WheelGripperCommand(WheelGripperPosition.SLOW_OUTTAKE));

		// -------------Arm Controls-------------
		m_armSubsystem.setDefaultCommand(new ManualMotorCommand(
				() -> m_operatorController1.getRawAxis(Axis.kLeftY),
				() -> m_operatorController1.getRawAxis(Axis.kRightY)));

		// Move the arm to the high node
		new JoystickButton(m_operatorController1, Button.kTriangle)
				.and(() -> !m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.HIGH_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.HIGH)));

		// Move the arm to the medium node over the back
		new JoystickButton(m_operatorController1, Button.kTriangle)
				.and(() -> m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.HIGH_BACK)));

		// Move the arm to the medium node
		new JoystickButton(m_operatorController1, Button.kSquare)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));

		// Move the arm to the medium node over the back
		new JoystickButton(m_operatorController1, Button.kCircle)
				.and(() -> m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK)));

		// Move the arm to the low position
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kX)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.LOW)));

		// Move the arm to the pocket
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kCircle)
				.and(() -> !m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.POCKET)));

		new JoystickButton(m_operatorController1, ControllerConstants.Button.kTrackpad)
				.onTrue(new ArmScoreCommand(ArmPosition.HOLD));

		// Move the arm to the substation position
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kSquare)
				.and(() -> m_operatorController1.getRawButton(Button.kLeftTrigger))
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.SUBSTATION)));

		// -------------LED signaling-------------
		// Signal for a cube
		new POVButton(m_operatorController1, ControllerConstants.DPad.kLeft)
				.onTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));

		// Signal for a cone
		new POVButton(m_operatorController1, ControllerConstants.DPad.kRight)
				.onTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		new POVButton(m_operatorController1, ControllerConstants.DPad.kUp)
				.onTrue(new LEDCommand(StatusCode.RAINBOW_PARTY_FUN_TIME));
		new JoystickButton(m_operatorController1, ControllerConstants.Button.kRightBumper)
				.onTrue(new LEDCommand(StatusCode.DEFAULT));
		// new POVButton(m_operatorController, ControllerConstants.DPad.kDown)
		// .onTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));

		// -------------Driver Controls-------------
		// Opening gripper/dropping game piece
		m_driverController1.cross().whileTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));

		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.whileTrue(new WheelGripperCommand(WheelGripperPosition.OUTTAKE));

		new JoystickButton(m_driverController, ControllerConstants.Button.kSquare)
				.whileTrue(new TurnTimeCommand(.75, false, 1000));
		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.whileTrue(new WheelGripperCommand(WheelGripperPosition.SLOW_OUTTAKE));

		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
				.whileTrue(new DriveBrakeModeCommand());
		// Driving
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger)));
		// Fine Turning
		new JoystickButton(m_driverController, ControllerConstants.Button.kRightBumper)
				.whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.0, () -> DriveConstants.kFineTurningSpeed));
		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
				.whileTrue(new DefaultDriveCommand(() -> 0.0, () -> DriveConstants.kFineTurningSpeed, () -> 0.0));

		// new JoystickButton(m_driverController, ControllerConstants.Button.kOptions)
		// .whileTrue(new AutoAlignmentCommand(new Pose(2, -2.65, Math.PI), 0.1, 2));
		// new JoystickButton(m_driverController, ControllerConstants.Button.kShare)
		// .whileTrue(new AutoAlignmentCommand(new Pose(3
		// , -2.65, Math.PI), 0.1, 2));
	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		// return new BalancePIDCommand();
		return m_autoChooser.getSelected();
	}
}
