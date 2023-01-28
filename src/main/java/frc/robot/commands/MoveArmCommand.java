// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class MoveArmCommand extends CommandBase {
	private double x;
	private double y;

	/** Creates a new MoveArmCommand. */
	public MoveArmCommand(double x, double y) {
		this.x = x;
		this.y = y;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Double[] e = InverseKinematicsTool.getArmAngles(x, y);

		double targetLowerArmAngle = (double) e[0];
		double targetUpperArmAngle = (double) e[1];
		SmartDashboard.putNumber("Target Lower Arm Angle", Math.toDegrees(targetLowerArmAngle));
		SmartDashboard.putNumber("Target Upper Arm Angle", Math.toDegrees(targetUpperArmAngle));
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
