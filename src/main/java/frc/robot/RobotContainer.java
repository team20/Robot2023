// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LEDs.LEDCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.arm.ChangeOffsetCommand;
import frc.robot.commands.drive.AutoAlignCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.gripper.GripperCommand;
import frc.robot.commands.gripper.GripperCommand.GripperPosition;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.util.CommandComposer;
import frc.robot.util.PoseMap;
import hlib.drive.AutoAligner;
import hlib.drive.AutoAlignerBasic;

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
	private final SendableChooser<Command> m_autoAlignChooser = new SendableChooser<>();
	
	private AutoAligner m_autoAligner = new AutoAlignerBasic(DriveConstants.kTrackwidthMeters, 0.1, 10 * Math.PI / 180,
			0.5, 0.1,
			0.08, new PoseMap(Filesystem.getDeployDirectory() + File.separator + "poses.json"));
	private PoseEstimationSubsystem m_poseSubsystem = new PoseEstimationSubsystem(DriveConstants.kTrackwidthMeters, 0.5,
			10, 0.1);

	public RobotContainer() {
		m_autoChooser.addOption("Out of Community", CommandComposer.getOutOfCommunityAuto(0));
		m_autoChooser.addOption("Onto Charge Station", CommandComposer.getOnToChargerAuto(0));
		m_autoChooser.addOption("Score 1 piece", CommandComposer.getScorePieceAuto());
		m_autoChooser.addOption("Leave then balance", CommandComposer.getLeaveThenBalanceAuto(1));// TODO fix distance
		m_autoChooser.addOption("Score then balance", CommandComposer.getScoreThenBalanceAuto());
		m_autoChooser.addOption("Score, leave over charge, balance", CommandComposer.getOverTheFulcrumAuto());
		m_autoChooser.addOption("Score two", CommandComposer.getTwoScoreAuto());
		m_autoChooser.addOption("Score two and balance", CommandComposer.getTwoScoreBalanceAuto());
		SmartDashboard.putData(m_autoChooser);

		m_autoAlignChooser.addOption("Move to Target 1", new AutoAlignCommand("1", m_autoAligner, m_poseSubsystem, 0.1));
		m_autoAlignChooser.addOption("Move to Target 2", new AutoAlignCommand("2", m_autoAligner, m_poseSubsystem, 0.1));
		m_autoAlignChooser.addOption("Move to Target 3", new AutoAlignCommand("3", m_autoAligner, m_poseSubsystem, 0.1));
		SmartDashboard.putData(m_autoAlignChooser);

		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// -------------Gripper Controls-------------
		new JoystickButton(m_operatorController, ControllerConstants.Button.kLeftBumper)
				.whileTrue(new SequentialCommandGroup(new GripperCommand(GripperPosition.CLOSE),
						(new LEDCommand(StatusCode.DEFAULT))));
		new JoystickButton(m_operatorController, ControllerConstants.Button.kRightBumper)
				.whileTrue(new GripperCommand(GripperPosition.OPEN));

		// -------------Arm Controls-------------
		m_armSubsystem.setDefaultCommand(new ChangeOffsetCommand(
				() -> m_operatorController.getRawAxis(ControllerConstants.Axis.kLeftX),
				() -> m_operatorController.getRawAxis(ControllerConstants.Axis.kRightY)));
		// Move the arm to the high node
		new JoystickButton(m_operatorController, ControllerConstants.Button.kTriangle)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.HIGH)));
		// Flip the arm over to the medium node

		new JoystickButton(m_operatorController, ControllerConstants.Button.kTrackpad)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_BACK)));
		// Move the arm to the medium node
		new JoystickButton(m_operatorController, ControllerConstants.Button.kSquare)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.MEDIUM_FORWARD)));
		// Move the arm to the low position
		new JoystickButton(m_operatorController, ControllerConstants.Button.kX)
				.onTrue(new DeferredCommand(() -> CommandComposer.createArmScoreCommand(ArmPosition.LOW)));

		// -------------LED signaling-------------
		// Signal for a cube
		new POVButton(m_operatorController, ControllerConstants.DPad.kLeft)
				.onTrue(new LEDCommand(StatusCode.BLINKING_PURPLE));
		// Signal for a cone
		new POVButton(m_operatorController, ControllerConstants.DPad.kRight)
				.onTrue(new LEDCommand(StatusCode.BLINKING_YELLOW));
		new POVButton(m_operatorController, ControllerConstants.DPad.kUp)
				.onTrue(new LEDCommand(StatusCode.DEFAULT));
		new POVButton(m_operatorController, ControllerConstants.DPad.kDown)
				.onTrue(new LEDCommand(StatusCode.MOVING_GREEN_AND_BLUE_GRADIENT));

		// -------------Driver Controls-------------
		// Opening gripper/dropping game piece
		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.whileTrue(new GripperCommand(GripperPosition.OPEN));
		// Driving
		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.whileTrue(new AutoAlignCommand("2", m_autoAligner, m_poseSubsystem, 0.1));

		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
				.whileTrue(new AutoAlignCommand("3", m_autoAligner, m_poseSubsystem, 0.1));

		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger)));

	}

	// TODO get auto command from auto chooser
	public Command getAutonomousCommand() {
		return null;
	}

	public void periodic() {
		m_poseSubsystem.periodic();
	}

}
