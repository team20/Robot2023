// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

// import java.time.Instant;

// import java.time.Duration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
	/**
	 * Distance to move
	 */
	private double m_distance;
	private double m_startDistanceLeft;
	private double m_startDistanceRight;
	// private Instant m_startTime;
	// private double m_speed;

	private ProfiledPIDController m_controller;

	/**
	 * Creates a new DriveDistanceCommand.
	 * 
	 * @param distance The distance to drive in meters
	 */
	public DriveDistanceCommand(double distance) {
		m_distance = distance;
		// m_speed = 0.4;
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Creates a new DriveDistanceCommand.
	 * 
	 * @param distance The distance to drive in meters
	 * @param speed    The speed to drive at
	 */
	public DriveDistanceCommand(double distance, double speed) {
		m_distance = distance;
		// m_speed = speed;
		addRequirements(DriveSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		DriveSubsystem.get().resetEncoders();
		// SmartDashboard.putNumber("encoder start left",
		// DriveSubsystem.get().getLeftEncoderPosition());
		// SmartDashboard.putNumber("encoder start right",
		// DriveSubsystem.get().getRightEncoderPosition());
		// m_startTime = Instant.now();

		// Creates a ProfiledPIDController
		// Max velocity is 5 meters per second
		// Max acceleration is 10 meters per second
		double kP = .1;
		double kI = 0.000;
		double kD = 0.00;

		m_controller = new ProfiledPIDController(kP, kI, kD,
				new TrapezoidProfile.Constraints(125, 150)); // was 196 35
		// m_startDistanceLeft = DriveSubsystem.get().getLeftEncoderPosition();
		// m_startDistanceRight= DriveSubsystem.get().getRightEncoderPosition();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_startDistanceLeft = 0;
		m_startDistanceRight = 0;
		// SmartDashboard.putNumber("encoder start left",
		// DriveSubsystem.get().getLeftEncoderPosition());
		// SmartDashboard.putNumber("encoder start right",
		// DriveSubsystem.get().getRightEncoderPosition());

		// Calculates the output of the PID algorithm based on the sensor reading
		// and sends it to a motor
		double output = m_controller.calculate(DriveSubsystem.get().getAverageEncoderDistance(), m_distance);
		DriveSubsystem.get().tankDrive(output, output);

		// DriveSubsystem.get().tankDrive(m_speed * Math.signum(m_distance), m_speed *
		// Math.signum(m_distance)); // TODO set
		// speeds
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0); // TODO set speeds
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		double currDistanceLeft = DriveSubsystem.get().getLeftEncoderPosition();
		double currDistanceRight = DriveSubsystem.get().getRightEncoderPosition();
		// SmartDashboard.putNumber("encoder current left",
		// DriveSubsystem.get().getLeftEncoderPosition());
		// SmartDashboard.putNumber("encoder current right",
		// DriveSubsystem.get().getRightEncoderPosition());
		// SmartDashboard.putBoolean("DriveDistance finished", (currDistanceLeft -
		// m_startDistanceLeft) > m_distance
		// && (currDistanceRight - m_startDistanceRight) > m_distance);

		// return Math.abs(currDistanceLeft - m_startDistanceLeft) >
		// Math.abs(m_distance)
		// && Math.abs(currDistanceRight - m_startDistanceRight) > Math.abs(m_distance);
		// return m_controller.atGoal();
		// If the distance measured by the encoders is more than the target distance,
		// stop the command
		return Math.abs(DriveSubsystem.get().getAverageEncoderDistance()) > Math.abs(m_distance);
	}
}