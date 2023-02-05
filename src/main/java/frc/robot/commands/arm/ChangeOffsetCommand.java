// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;
import frc.robot.util.InverseKinematicsTool;

public class ChangeOffsetCommand extends CommandBase {
	// Saved value for how much to move
	double m_xOffset;
	double m_yOffset;
	private Supplier<Double> m_joystickY;
	private Supplier<Double> m_joystickX;

	/** Creates a new ChangeOffsetCommand. */
	public ChangeOffsetCommand(Supplier<Double> joystickY, Supplier<Double> joystickX) {
		m_joystickY = joystickY;
		m_joystickX = joystickX;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_yOffset = m_joystickY.get();
		m_xOffset = m_joystickX.get();
		// Get current position from angles
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getUpperArmPosition(),
				ArmSubsystem.get().getLowerArmPosition());
		// Add xOffset and yOffset to that position
		double newX = coordinates[0] + m_xOffset;
		double newY = coordinates[1] + m_yOffset;
		// Logging
		SmartDashboard.putNumber("newX", newX);
		SmartDashboard.putNumber("newY", newY);
		// Calculate angles for new position
		Double[] armPosition = InverseKinematicsTool.getArmAngles(newX, newY);
		// set angles
		if (armPosition != null) {
			ArmSubsystem.get().setLowerArmPosition(armPosition[0]);
			ArmSubsystem.get().setUpperArmPosition(armPosition[1]);
		}
	}

}
