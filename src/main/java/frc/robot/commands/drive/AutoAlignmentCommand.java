// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import hlib.drive.Pose;

/**
 * An {@code AutoAlignmentCommand} can automatically align the robot to a
 * certain target {@code Pose}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AutoAlignmentCommand extends SequentialCommandGroup {

	/**
	 * The target {@code Pose}.
	 */
	Pose m_target;

	/**
	 * The distance threshold.
	 */
	double m_distanceThreshold;

	/**
	 * The angle threshold.
	 */
	double m_angleThreshold;

	/**
	 * The start time of this {@code AutoAlignmentCommand}.
	 */
	long m_startTime;

	/**
	 * Constructs an {@code AutoAlignmentCommand}.
	 * 
	 * @param target
	 *                          the target {@code Pose}
	 * @param distanceThreshold
	 *                          the distance threshold (in meters)
	 * @param angleThreshold
	 *                          the angle threshold (in degrees)
	 */
	public AutoAlignmentCommand(Pose target, double distanceThreshold, double angleThreshold) {
		m_target = target;
		m_distanceThreshold = distanceThreshold;
		m_angleThreshold = angleThreshold;
		addRequirements(DriveSubsystem.get());
		addCommands(new CommandBase() {
			public void initialize() {
				SmartDashboard.putString("Target", "" + m_target);
				m_startTime = System.currentTimeMillis();
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		});
		addCommands(new TurnToTargetCommand());
		addCommands(new MoveToTargetCommand().withTimeout(5));
		addCommands(new AlignToTargetCommand());
		addCommands(new CommandBase() {
			@Override
			public boolean isFinished() {
				return true;
			}

			public void end(boolean interrupted) {
				SmartDashboard.putString("Target", "");
				double duration = 0.001 * (System.currentTimeMillis() - m_startTime);
				SmartDashboard.putString("Last Travel Time",
						String.format("%.2f seconds to target %s", duration, m_target));
			}
		});
	}

	/**
	 * An {@code Aligner} performs some alignment task for the robot.
	 */
	interface Aligner {

		/**
		 * Performs some alignment task for the robot.
		 * 
		 * @param displacement        the positional displacement of the target
		 *                            {@code Pose} compared to the current {@code Pose}
		 *                            of the robot
		 * @param angularDisplacement the angular displacement of the target
		 *                            {@code Pose} compared to the current {@code Pose}
		 *                            of the robot (in radians)
		 */
		public void align(double displacement, double angularDisplacement);

	}

	/**
	 * Aligsn the robot using the specified {@code Aligner}.
	 * 
	 * @param aligner an {@code Aligner}
	 */
	void align(Aligner aligner) {
		Pose poseEstimated = PoseEstimationSubsystem.get().poseEstimated();
		if (poseEstimated != null) {
			double displacement = poseEstimated.distance(m_target);
			double angularDisplacement = Pose
					.normalize(poseEstimated.angleTo(m_target) - poseEstimated.directionalAngle());
			boolean backwardAlginment = Math.abs(angularDisplacement) > Math.PI / 2;
			if (backwardAlginment) { // if backward alignment
				displacement = -displacement;
				angularDisplacement = Pose.normalize(angularDisplacement - Math.PI);
			}
			aligner.align(displacement, angularDisplacement);
		} else
			DriveSubsystem.get().tankDrive(0, 0);

	}

	/**
	 * A {@code AlignToTargetCommand} can turn the robot so that it matches the
	 * direction of the target {@code Pose}.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	class AlignToTargetCommand extends CommandBase {

		/**
		 * The {@code PIDController} used by this {@code AlignToTargetCommand}.
		 */
		PIDController m_controller;

		/**
		 * Is called when this {@code AlignToTargetCommand} is initially scheduled.
		 */
		@Override
		public void initialize() {
			m_controller = new PIDController(DriveConstants.kTurnP,
					DriveConstants.kTurnI, DriveConstants.kTurnD); // parameters copied from DriveDistanceCommand
			m_controller.enableContinuousInput(-180, 180);
			m_controller.setSetpoint(0);
			m_controller.setTolerance(m_angleThreshold);
		}

		/**
		 * Is called every time the scheduler runs this {@code AlignToTargetCommand}.
		 */
		@Override
		public void execute() {
			Pose poseEstimated = PoseEstimationSubsystem.get().poseEstimated();
			if (poseEstimated != null) {
				double angularDisplacement = (m_target.directionalAngle() - poseEstimated.directionalAngle()) * 180
						/ Math.PI;
				turn(angularDisplacement);
			} else
				DriveSubsystem.get().tankDrive(0, 0);
		}

		/**
		 * Turns the robot to reduce the angular displacement (in degrees).
		 * 
		 * @param angularDisplacement the angular displacement (in degrees)
		 */
		void turn(double angularDisplacement) {
			double turnSpeed = m_controller.calculate(angularDisplacement);
			// if turn speed is less than 0.1 make it 0.1 in the right direction
			turnSpeed = Math.abs(turnSpeed) < 0.15 ? Math.signum(turnSpeed) * 0.15 : turnSpeed;
			turnSpeed = MathUtil.clamp(turnSpeed, -0.5, 0.5);
			SmartDashboard.putString("Wheel Velocities",
					String.format("(%.3f, %.3f)", turnSpeed, -turnSpeed));
			SmartDashboard.putNumber( "Angular Displacement", angularDisplacement);
			DriveSubsystem.get().tankDrive(turnSpeed, -turnSpeed);
		}

		/**
		 * Is called once this {@code AlignToTargetCommand} ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			DriveSubsystem.get().tankDrive(0, 0);
		}

		/**
		 * Determines whether or not this {@code AlignToTargetCommand} meets the
		 * criteria
		 * for completion.
		 * 
		 * @return {@code true} if this {@code AlignToTargetCommand} meets the criteria
		 *         for
		 *         completion; {@code false} otherwise
		 */
		@Override
		public boolean isFinished() {
			return m_controller.atSetpoint();
		}

	}

	/**
	 * A {@code TurnToTargetCommand} can turn the robot to the target {@code Pose}.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	class TurnToTargetCommand extends AlignToTargetCommand {

		/**
		 * Is called every time the scheduler runs this {@code TurnToTargetCommand}.
		 */
		@Override
		public void execute() {
			align((displacement, angularDisplacement) -> {
				turn(angularDisplacement * 180 / Math.PI);
			});
		}
	}

	/**
	 * A {@code MoveToTargetCommand} can move the robot to the target {@code Pose}.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	class MoveToTargetCommand extends CommandBase {

		/**
		 * The {@code ProfiledPIDController} used by this {@code MoveToTargetCommand}.
		 */
		ProfiledPIDController m_controller;

		/**
		 * Is called when this {@code MoveToTargetCommand} is initially scheduled.
		 */
		@Override
		public void initialize() {
			m_controller = new ProfiledPIDController(0.3, 0, 0,
					new TrapezoidProfile.Constraints(3, 2)); // parameters copied from DriveDistanceCommand
		}

		/**
		 * Is called every time the scheduler runs this {@code MoveToTargetCommand}.
		 */
		@Override
		public void execute() {
			align((displacement, angularDisplacement) -> {
				double velocity = m_controller.calculate(0, displacement);
				// double velocity = m_controller.calculate(-displacement, 0);
				double[] velocities = new double[] { (angularDisplacement < 0 ? 1 : 0.9) * velocity,
						(angularDisplacement > 0 ? 1 : 0.9) * velocity };
				SmartDashboard.putString("Wheel Velocities",
						String.format("(%.3f, %.3f)", velocities[0], velocities[1]));
				DriveSubsystem.get().tankDrive(velocities[0], velocities[1]);
			});
		}

		/**
		 * Is called once this {@code MoveToTargetCommand} ends or is interrupted.
		 */
		@Override
		public void end(boolean interrupted) {
			DriveSubsystem.get().tankDrive(0, 0);
		}

		/**
		 * Determines whether or not this {@code MoveToTargetCommand} meets the criteria
		 * for completion.
		 * 
		 * @return {@code true} if this {@code MoveToTargetCommand} meets the criteria
		 *         for
		 *         completion; {@code false} otherwise
		 */
		@Override
		public boolean isFinished() {
			Pose poseEstimated = PoseEstimationSubsystem.get().poseEstimated();
			return poseEstimated == null || poseEstimated.distance(m_target) < m_distanceThreshold;
		}

	}

}
