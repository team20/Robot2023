// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagLimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A {@code DriveToApriltag} can move the robot by a certain distance.
 * 
 * @author Jeong Hyon Hwang
 */

public class DriveToApriltag extends CommandBase {
	
	private double m_speed;

	public DriveToApriltag(double speed) {
		
		m_speed = speed;
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is called when this {@code DriveToApriltag} is initially scheduled.
	 */
	@Override
	public void initialize() {
		DriveSubsystem.get().resetEncoders();
	}

	/**
	 * Is called every time the scheduler runs while this
	 * {@code DriveToApriltag} is scheduled.
	 */
	@Override
	public void execute() {
		double angle = AprilTagLimelightSubsystem.get().getTX();

		double turn = angle / (30);

		DriveSubsystem.get().arcadeDrive(m_speed, turn*2);
	}

	/**
	 * Is called once this {@code DriveToApriltag} ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
	}

	/**
	 * Determines whether or not this {@code DriveToApriltag} is completed.
	 * 
	 * @return {@code true} if this {@code DriveToApriltag} is completed;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return AprilTagLimelightSubsystem.get().getDistance() > -0.75;
	}
}
