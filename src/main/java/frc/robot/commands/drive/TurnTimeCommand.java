// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnTimeCommand extends CommandBase {
	Instant m_startTime;
	private double m_time;
	private double m_clockwise;

	/**
	 * Creates a new TurnTimeCommand.
	 * 
	 * @param clockwise Whether we're going clockwise or counterclockwise
	 * @param time      Time in seconds to turn
	 */
	public TurnTimeCommand(boolean clockwise, double time) {
		m_clockwise = clockwise ? 1 : -1;
		m_time = time;
		addRequirements(DriveSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_startTime = null;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_startTime == null) {
			m_startTime = Instant.now();
		}
		DriveSubsystem.get().tankDrive(-0.1 * m_clockwise, 0.1 * m_clockwise);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Duration.between(m_startTime, Instant.now()).toSeconds() > m_time;
	}
}
