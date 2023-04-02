// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ManualMotorCommand extends CommandBase {
	/** Stores a method to get the joystick input for the lower arm */
	private Supplier<Double> m_lowerArmInput;
	/** Stores a method to get the joystick input for the upper arm */
	private Supplier<Double> m_upperArmInput;

	/**
	 * Stores the lower arm motor speed after deadbands and speed multipliers are
	 * applied to joystick input
	 */
	private double lowerArmMotorSpeed;
	/**
	 * Stores the upper arm motor speed after deadbands and speed multipliers are
	 * applied to joystick input
	 */
	private double upperArmMotorSpeed;

	/**
	 * Creates a new ManualMotorCommand.
	 */
	public ManualMotorCommand(Supplier<Double> lowerArmInput, Supplier<Double> upperArmInput) {
		// When the operator and the robot(front side) are facing each other, left on
		// the joystick should move the lower arm back, and right should move the lower
		// arm forward. Up and down should move the upper arm accordingly.
		m_lowerArmInput = lowerArmInput;
		m_upperArmInput = upperArmInput;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		lowerArmMotorSpeed = -MathUtil.applyDeadband(m_lowerArmInput.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kArmMotorSpeedSensitivity;
		upperArmMotorSpeed = -MathUtil.applyDeadband(m_upperArmInput.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kArmMotorSpeedSensitivity;

		if(lowerArmMotorSpeed != 0 || upperArmMotorSpeed!=0 || ArmSubsystem.get().getManualArmRan()){
			ArmSubsystem.get().setLowerArmMotorSpeed(-lowerArmMotorSpeed);
			ArmSubsystem.get().setUpperArmMotorSpeed(upperArmMotorSpeed);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// If for some reason the command ends, stop the motors
		ArmSubsystem.get().setLowerArmMotorSpeed(0);
		ArmSubsystem.get().setUpperArmMotorSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
