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
	private double m_newX;
	private double m_newY;
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
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getUpperArmAngle(),
				ArmSubsystem.get().getLowerArmAngle());
		m_newX = coordinates[0];
		m_newY = coordinates[1];
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Amount to move on the x-axis
		m_xOffset = MathUtil.applyDeadband(m_joystickX.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kSpeedMultiplier;
		// The -1 is because the y-axis values are inverted
		// Amount to move on the y-axis
		m_yOffset = MathUtil.applyDeadband(m_joystickY.get(), ControllerConstants.kDeadzone)
				* ArmConstants.kSpeedMultiplier * -1;
		// Get current (X, Y) position from current actual angles
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getUpperArmAngle(),
				ArmSubsystem.get().getLowerArmAngle());
		// If moving on the x-axis, add m_xOffset to the current x-coordinate,
		// otherwise, do nothing
		// If moving on the y-axis, add m_yOffset to the current y-coordinate,
		// otherwise, do nothing
		// The reason for the if statements is to prevent movement on another axis when
		// only one axis is being moved. Otherwise, the arm will naturally move due to
		// gravity, causing both axes to change when we want only one to be changed
		if ((m_xOffset != 0) || !ArmSubsystem.get().isNearTargetAngle()) {
			m_newX = coordinates[0] + m_xOffset;
		}
		if ((m_yOffset != 0) || !ArmSubsystem.get().isNearTargetAngle()) {
			m_newY = coordinates[1] + m_yOffset;
		}

		// Logging
		SmartDashboard.putNumber("newX", m_newX);
		SmartDashboard.putNumber("newY", m_newY);
		// Calculate angles for new position
		Double[] armPosition = InverseKinematicsTool.calculateArmAngles(m_newX, m_newY);
		if (armPosition != null && armPosition[0] > 100) {
			// Prevent going more than 10 degrees past vertical
			armPosition = null;
		}
		if (m_newY > 12) {
			// Height limit!
			armPosition = null;
		}
		// Set angles, if they are invalid, do nothing
		if (armPosition != null) {
			// If the sign of the current arm position is different than the sign of the
			// target arm position (positive/negative X transition), move the upper arm
			// first, then the lower arm to reduce sudden snapping
			if (false && Math.signum(coordinates[0]) != Math.signum(m_newX)) {
				// Get the number of degrees the lower arm will travel
				double lowerArmAngleDiff = 180 - armPosition[0] * 2;
				// If we are going negative, we need to add the degrees the lower arm will
				// travel to current upper arm angle
				if (m_newX == -1) {
					// Keep the upper arm at the same angle relative to the arm base
					ArmSubsystem.get().setUpperArmAngle(ArmSubsystem.get().getUpperArmAngle() + lowerArmAngleDiff);
					// If we are going positive, we need to subtract the degrees the lower arm will
					// travel from the current upper arm angle
				} else if (m_newX == 1) {
					ArmSubsystem.get().setUpperArmAngle(ArmSubsystem.get().getUpperArmAngle() - lowerArmAngleDiff);
				}

				// Move the lower arm into position
				ArmSubsystem.get().setLowerArmAngle(armPosition[0]);
			}
			// When we are done getting the upper arm into the intermediate position, set it
			// to the calculated angle
			SmartDashboard.putNumber("Target Lower Arm Angle", armPosition[0]);
			SmartDashboard.putNumber("Target Upper Arm Angle", armPosition[1]);
			ArmSubsystem.get().setLowerArmAngle(armPosition[0]);
			ArmSubsystem.get().setUpperArmAngle(armPosition[1]);
		}
	}
}
