// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.StatusCode;

public class LEDCommand extends CommandBase {
	private StatusCode m_StatusCode;

	/** Creates a new LEDCommand. */
	public LEDCommand(StatusCode code) {
		m_StatusCode = code;
		addRequirements(LEDSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		LEDSubsystem.get().setCode(m_StatusCode);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
