// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ForwardKinematicsTool;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class ChangeOffsetCommand extends CommandBase {
	/** Creates a new ChangeOffsetCommand. */
	double[] coordinates = ForwardKinematicsTool.getArmPosition();
	double xOffset = coordinates[0];
	double yOffset = coordinates[1];

	public ChangeOffsetCommand() {
		final GenericHID m_controller = new GenericHID(frc.robot.Constants.ControllerConstants.kDriverControllerPort);
		addRequirements(ArmSubsystem.get());

		xOffset += m_controller.getRawAxis(0);
		yOffset += m_controller.getRawAxis(1);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		InverseKinematicsTool.getArmAngles(xOffset, yOffset);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
