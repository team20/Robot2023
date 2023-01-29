// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.InverseKinematicsTool;

public class MoveArmCommand extends CommandBase {
	private double m_x;
	private double m_y;

	/** Creates a new MoveArmCommand. */
	public MoveArmCommand(double x, double y) {
		m_x = x;
		m_y = y;
		addRequirements(ArmSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Double[] e = InverseKinematicsTool.getArmAngles(m_x, m_y);

		double targetLowerArmAngle = (double) e[0];
		double targetUpperArmAngle = (double) e[1];
		SmartDashboard.putNumber("Target Lower Arm Angle", Math.toDegrees(targetLowerArmAngle));
		SmartDashboard.putNumber("Target Upper Arm Angle", Math.toDegrees(targetUpperArmAngle));
		ArmSubsystem.get().setLowerArmPosition(Math.toDegrees(targetLowerArmAngle));
		// ArmSubsystem.get().setUpperArmPosition(targetUpperArmAngle);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// double lowerArmPosition = ArmSubsystem.get().getLowerArmPosition();
		// if(lowerArmPosition >=90){
		// return true;
		// } else{
		// return false;
		// }
		// }
		return false;
	}
}
