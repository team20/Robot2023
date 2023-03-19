// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Used to delay command construction from robot startup(during
 * configureButtonBindings) to command scheduling(when a Trigger or one of its
 * subclasses' condition is met.) Useful for when the command that is executed
 * changes depending on runtime conditions.
 * 
 * @author Jonathan Waters
 */
public class DeferredCommand extends CommandBase {
	private final Supplier<Command> m_commandSupplier;
	private Command m_command;

	/**
	 * Creates a new DeferredCommand.
	 * 
	 * @param commandSupplier An object that gives back a Command. Usually a lambda
	 *                        expression that calls a method that returns a Command.
	 *                        This method should have logic to change the command
	 *                        returned based on some selected condition(s)
	 */
	public DeferredCommand(Supplier<Command> commandSupplier) {
		m_commandSupplier = commandSupplier;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Obtain the command
		m_command = m_commandSupplier.get();
		m_command.schedule();
	}


	@Override public void end(boolean interrupted){
		if(interrupted){
			m_command.cancel();
		}
		m_command = null;
	}
	@Override 
	public void execute(){

	}
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		//return true;
		return !m_command.isScheduled();
	}
}