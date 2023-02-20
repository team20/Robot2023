// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalancePIDCommand extends CommandBase {
	PIDController m_controller = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI,
			DriveConstants.kBalanceD);

	/** Creates a new BalancePIDCommand. */
	public BalancePIDCommand() {
		addRequirements(DriveSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_controller.setSetpoint(0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double setpoint = m_controller.calculate(DriveSubsystem.get().getPitch());
		DriveSubsystem.get().tankDrive(setpoint, setpoint);
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
