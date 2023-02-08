// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ForwardKinematicsTool;
import frc.robot.util.InverseKinematicsTool;

public class ChangeOffsetCommand extends CommandBase {
	// Saved value for how much to move
	private double m_xOffset;
	private double m_yOffset;
	private Supplier<Double> m_joystickX;
	private Supplier<Double> m_joystickY;

	/** Creates a new ChangeOffsetCommand. */
	public ChangeOffsetCommand(Supplier<Double> joystickX, Supplier<Double> joystickY) {
		m_joystickX = joystickX;
		m_joystickY = joystickY;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_xOffset = MathUtil.applyDeadband(m_joystickX.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kSpeedMultiplier;
		// The -1 is because the Yaxis values are inverted
		m_yOffset = MathUtil.applyDeadband(m_joystickY.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kSpeedMultiplier * -1;
		// m_xOffset = m_joystickX.get();
		// m_yOffset = m_joystickY.get();
		// Get current position from angles
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getUpperArmAngle(),
				ArmSubsystem.get().getLowerArmAngle());
		// Add xOffset and yOffset to that position
		double newX = coordinates[0] + m_xOffset;
		double newY = coordinates[1] + m_yOffset;
		// Logging
		SmartDashboard.putNumber("newX", newX);
		SmartDashboard.putNumber("newY", newY);
		// Calculate angles for new position
		Double[] armPosition = InverseKinematicsTool.calculateArmAngles(newX, newY);
		// Set angles, if they are invalid, do nothing
		if (armPosition != null) {
			SmartDashboard.putNumber("Target Lower Arm Angle", armPosition[0]);
			SmartDashboard.putNumber("Target Upper Arm Angle", armPosition[1]);
			ArmSubsystem.get().setLowerArmAngle(armPosition[0]);
			ArmSubsystem.get().setUpperArmAngle(armPosition[1]);
		}
	}
}
