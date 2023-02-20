// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
	/** Creates a new ArmCommand. */
	public enum Operation {
		CMD_ARM_UP,
		CMD_ARM_DOWN,
		CMD_ARM_SETTLE,
		CMD_RESET_ENCODER,
		CMD_ARM_MANUAL,
		CMD_ARM_STOP,
		CMD_ARM_MANUAL_DOWN
	}

	private Operation m_operation;

	public ArmCommand(Operation operation) {
		m_operation = operation;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		System.out.println("scheduling arm");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		/*
		 * if(m_operation == Operation.CMD_ARM_UP){
		 * ArmSubsystem.get().setPosition(ArmSubsystem.Position.UP_POSITION);
		 * ArmSubsystem.get().setBrakeMode();
		 * //System.out.println("setting to up");
		 * // ArmSubsystem.get().setPercentOutput(0);//TODO find speed
		 * } else if(m_operation == Operation.CMD_ARM_DOWN){
		 * //System.out.println("setting to down");
		 * ArmSubsystem.get().setPosition(ArmSubsystem.Position.DOWN_POSITION);
		 * ArmSubsystem.get().setCoastMode();
		 * // ArmSubsystem.get().setPercentOutput(0);//TODO find speed
		 * }
		 * else if (m_operation==Operation.CMD_ARM_MANUAL_DOWN) {
		 * ArmSubsystem.get().setPercentOutput(.5);
		 * }
		 * else if(m_operation==Operation.CMD_RESET_ENCODER){
		 * ArmSubsystem.get().resetEncoder();
		 * }
		 * else if(m_operation==Operation.CMD_ARM_MANUAL){
		 * ArmSubsystem.get().setPercentOutput(-.6);
		 * }
		 * else if(m_operation==Operation.CMD_ARM_STOP){
		 * ArmSubsystem.get().setPercentOutput(0);
		 * }
		 */

		ArmSubsystem.get().setSpeedLower(.2);

	}
}