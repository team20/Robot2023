// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
	/**
	 * Distance to move
	 */
	private double m_distance;
	private double m_startDistanceAverage;
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
		// Creates a ProfiledPIDController
		// Max velocity is 5 meters per second
		// Max acceleration is 10 meters per second
		double kP = .1;
		double kI = 0.000;
		double kD = 0.00;

		//TODO fix Trapezoidal profiles
		m_controller = new ProfiledPIDController(kP, kI, kD,
				new TrapezoidProfile.Constraints(125, 150)); // was 196 35
		m_startDistanceAverage = DriveSubsystem.get().getAverageEncoderDistance();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Calculates the output of the PID algorithm based on the sensor reading
		// and sends it to a motor
		double output = m_controller
				.calculate(DriveSubsystem.get().getAverageEncoderDistance() - m_startDistanceAverage, m_distance);
		DriveSubsystem.get().tankDrive(output, output);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0); // TODO set speeds
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(DriveSubsystem.get().getAverageEncoderDistance() - m_startDistanceAverage) > Math
				.abs(m_distance);
	}
}