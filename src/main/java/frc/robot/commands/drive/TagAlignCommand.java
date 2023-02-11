// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TagAlignCommand extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  //PIDController m_controller = new PIDController(0.14, 0.005, 0.000); 
  //i: 0.005 before d: 0.000 before
  private double m_driveSpeed = 0.15;  //constant
  private double m_speed = 0.15; //changes
  private double m_xOffset, m_zOffset; //offsets from april tag

  private Pose2d m_goalPose = new Pose2d(); //instantiates pose of where robot is
  private double m_vectorDistance; //Vector distance from robot to April tag
  public enum TagNumber{ //method of constants for april tag offsets
    TagId1(1,-1,0),
    TagId2(1,-1,0),
    TagId3(1,-1,0),
    TagId4(1,-1,0),
    TagId5(1,-1,0),
    TagId6(1,-1,0),
    TagId7(1,-1,0),
    TagId8(1,-1,0);

    public double yOffsetMiddle;
    public double yOffsetLeft;
    public double yOffsetRight;

    private TagNumber(double offsetLeft, double offsetRight, double offsetMiddle){ //sets global offsets to that of specific the april tag
      yOffsetLeft = offsetLeft;
      yOffsetRight = offsetRight;
      yOffsetMiddle = offsetMiddle;
    }
  }

  public enum Position{ //variables for 2nd constructor, what the offset is
    LeftPosition,
    RightPosition,
    MiddlePosition;
  }
  public TagAlignCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSubsystem.get());
  }
  /*
   * xOffset = offset parallel to the tag plane
   * zOffset = offset perpendicular to the tag plane
   */
  public TagAlignCommand(double xOffset, double zOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xOffset = xOffset;
    m_zOffset = zOffset;
    addRequirements(DriveSubsystem.get());
  }
  public TagAlignCommand(TagNumber tagId, Position pos, double zOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    switch(pos){ //switch for position based on parameter
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
  //DriveSubsystem.get().tankDrive(-0.25, -0.25);
  double z = -AprilTagSubsystem.get().getDistance();
  double x = AprilTagSubsystem.get().getX();

  Pose2d currPose = DriveSubsystem.get().getPose(); //current pose based on navx data
  m_goalPose = currPose.transformBy(new Transform2d(new Translation2d(z+m_zOffset, x+m_xOffset), new Rotation2d()));

  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = DriveSubsystem.get().getPose();

    double z = -AprilTagSubsystem.get().getDistance(); //distance out of april tag is negative, fix that
    double x = AprilTagSubsystem.get().getX();

    if(z != 0){
      //m_goalPose = currPose.transformBy(new Transform2d(new Translation2d(z+m_zOffset, x+m_xOffset), new Rotation2d()));
    }

    DriveSubsystem.get().arcadeDrive(m_driveSpeed, getTurn(m_goalPose, DriveSubsystem.get().getPose())); //call arcade drive with goalpose and turn 
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    DriveSubsystem.get().tankDrive(0, 0); //stop robot
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_goalPose.getX() - DriveSubsystem.get().getPose().getX()) < 0.1) { //if we've reached the goal
      return true;
    }
    return false;
  }

  private double getTurn(Pose2d goalPoint, Pose2d currPoint) {

    SmartDashboard.putNumber("Goal X", goalPoint.getX());
    SmartDashboard.putNumber("Goal Y", goalPoint.getY());
    SmartDashboard.putNumber("Curr X", currPoint.getX());
    SmartDashboard.putNumber("Curr Y", currPoint.getY());
    // just the distance formula
    double xDifference = goalPoint.getX() - currPoint.getX();
    double yDifference = goalPoint.getY() - currPoint.getY();
    m_vectorDistance = Math.sqrt(xDifference*xDifference + yDifference*yDifference);

    double vectorAngle = Math.atan2(yDifference,xDifference); //gets the angle of the vector from robot to tag
    double robotAngle = DriveSubsystem.get().getHeading(); //gets the direction the robot is pointed
    robotAngle = robotAngle < 0 ? robotAngle +360 : robotAngle; //converts -180 -> 180 degree angles to 0 -> 360 degree angles
    double angleChangeRadians = Math.toRadians(robotAngle); //to radians
    double vectorAngleAdjusted = vectorAngle - angleChangeRadians; //Subtract where the robot is pointed from how much it needs to turn
    double x = m_vectorDistance * Math.cos(vectorAngleAdjusted); //Gets distance perpendicular to tag
    SmartDashboard.putNumber("Vector Angle Adjusted", Math.toDegrees(vectorAngleAdjusted));
    SmartDashboard.putNumber("Vector Angle", Math.toDegrees(vectorAngle));
    SmartDashboard.putNumber("Robot Angle", Math.toDegrees(angleChangeRadians));

    if(x>0){ //Checks if the tag is infront of the bot
      m_driveSpeed = m_speed;
      SmartDashboard.putNumber("Turn Amount", m_speed*Math.sin(vectorAngleAdjusted));
      return Math.sin(vectorAngleAdjusted)*2;
    }else{
      m_driveSpeed = 0; //stops the bot
      return Math.signum(Math.sin(vectorAngleAdjusted)); //returns -1.0
    }

  }
}
