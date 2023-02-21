// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TagAlignCommand extends CommandBase {

	private double m_driveSpeed = LimelightConstants.kSpeed; 
	private double m_xOffset, m_zOffset; // offsets from april tag

	private Pose2d m_goalPose = new Pose2d(); // instantiates pose of where robot is

	public enum TagNumber { // method of constants for april tag offsets
		TagGeneral(0.5, -0.5, 0),
		TagId1(1, -1, 0),
		TagId2(1, -1, 0),
		TagId3(1, -1, 0),
		TagId4(1, -1, 0),
		TagId5(1, -1, 0),
		TagId6(1, -1, 0),
		TagId7(1, -1, 0),
		TagId8(1, -1, 0);

		public double yOffsetMiddle;
		public double yOffsetLeft;
		public double yOffsetRight;

		// sets global offsets to that of specific the april tag
		private TagNumber(double offsetLeft, double offsetRight, double offsetMiddle) {
			yOffsetLeft = offsetLeft;
			yOffsetRight = offsetRight;
			yOffsetMiddle = offsetMiddle;
		}
	}

	public enum Position { // variables for 2nd constructor, what the offset is
		LeftPosition,
		RightPosition,
		MiddlePosition;
	}

	/*
	 * Line up to apriltag directly
	 */
	public TagAlignCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(DriveSubsystem.get());
	}

	/*
	 * xOffset = offset parallel to the tag plane and floor plane
	 * zOffset = offset perpendicular to the tag plane
	 */
	public TagAlignCommand(double xOffset, double zOffset) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_xOffset = xOffset;
		m_zOffset = zOffset;
		addRequirements(DriveSubsystem.get());
	}

	/*
	 * Use offsets based on the tag and what position we want to go to
	 */
	public TagAlignCommand(TagNumber tagId, Position pos, double zOffset) {
		// Use addRequirements() here to declare subsystem dependencies.
		switch (pos) { // switch for position based on parameter
			case LeftPosition:
				m_xOffset = tagId.yOffsetLeft;
				break;
			case RightPosition:
				m_xOffset = tagId.yOffsetRight;
				break;
			case MiddlePosition:
				m_xOffset = tagId.yOffsetMiddle;
				break;

		}
		m_zOffset = zOffset;
		addRequirements(DriveSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// rotation math to translate the apriltag data + offsets into a field location
		// in the odometry frame
		// get the pose we intend to reach by the end of the command
		double z = AprilTagSubsystem.get().getDistance() + m_zOffset;
		double x = AprilTagSubsystem.get().getX() + m_xOffset;
		double aprilTagYaw = AprilTagSubsystem.get().getYaw();
		Pose2d currPose = DriveSubsystem.get().getPose();
		double currHeading = DriveSubsystem.get().getHeading();
		currHeading = currHeading < 0 ? currHeading + 360 : currHeading;
		double rotationToField = Math.toRadians(-aprilTagYaw) - currPose.getRotation().getRadians();
		Translation2d goalTranslation = (new Translation2d(-z, -x)).rotateBy(new Rotation2d(rotationToField));
		m_goalPose = new Pose2d(currPose.getX() + goalTranslation.getX(), currPose.getY() - goalTranslation.getY(),
				currPose.getRotation());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Goal X", m_goalPose.getX());
		SmartDashboard.putNumber("Goal Y", m_goalPose.getY());
		SmartDashboard.putNumber("Curr X", DriveSubsystem.get().getPose().getX());
		SmartDashboard.putNumber("Curr Y", DriveSubsystem.get().getPose().getY());

		// drive based on turns computed from our current position and our goal position
		DriveSubsystem.get().arcadeDrive(m_driveSpeed, getTurn(m_goalPose, DriveSubsystem.get().getPose()));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// stop the drive train when we're done
		DriveSubsystem.get().tankDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// end when we reach our goal point
		return Math.abs(m_goalPose.getX() - DriveSubsystem.get().getPose().getX()) < LimelightConstants.zTolerance
				&& Math.abs(m_goalPose.getY() - DriveSubsystem.get().getPose().getY()) < LimelightConstants.xTolerance;
	}

	public double getTurn(Pose2d goalPoint, Pose2d currPoint){
		SmartDashboard.putNumber("Goal Y", goalPoint.getY());
		SmartDashboard.putNumber("Curr X", currPoint.getX());
		SmartDashboard.putNumber("Curr Y", currPoint.getY());

		// just the distance formula
		double xDifference = goalPoint.getX() - currPoint.getX();
		double yDifference = goalPoint.getY() - currPoint.getY();
		double distanceSquared = xDifference * xDifference + yDifference * yDifference;
		// get the angle of the vector between our current robot location and our goal
		// point in the odometry frame
		// and adjust it to be in the robot frame
		double vectorAngle = Math.atan2(yDifference, xDifference);
		double robotAngle = DriveSubsystem.get().getHeading();
		robotAngle = robotAngle < 0 ? robotAngle + 360 : robotAngle;
		double angleChangeRadians = Math.toRadians(robotAngle);
		double vectorAngleAdjusted = vectorAngle - angleChangeRadians;
		// compute drive speed and turn angle using the angle,
		// half speed if we are less than half a meter from our goal point
		// turn speed is in relation to the sine of the computed angle
		// since the y-axis of the robot frame is parallel to the wheel axle
		// this means that our turn speed is the greatest when the goal point is
		// directly next to us
		// and least when the goal point is directly in front of us
		// drive speed is in relation to the cosine of the computed angle
		// this means that our speed is the greatest when the goal point is directly in
		// front of us
		// and least when the goal point is directly next to us
		double speedLimitFactor = (distanceSquared > LimelightConstants.kSlowDownDistanceSquared ? 1 : 0.5);
		m_driveSpeed = LimelightConstants.kSpeed * Math.cos(vectorAngleAdjusted) * speedLimitFactor;
		return Math.sin(vectorAngleAdjusted) * speedLimitFactor;
	}
}
