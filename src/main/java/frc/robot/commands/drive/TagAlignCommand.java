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
<<<<<<< Updated upstream
  private double m_driveSpeed = 0.15;  //constant
  private double m_speed = 0.15; //changes
  private double m_xOffset, m_zOffset; //offsets from april tag

=======
<<<<<<< HEAD
  private double m_driveSpeed = 0.3;
  private double m_speed = 0.3;
  private double m_xOffset, m_zOffset;

  private Pose2d m_goalPose = new Pose2d();

  //Store left, right, and center x-offsets(parallel to apriltag plane and floor plane)
  public enum TagNumber{
    TagGeneral(0.5, -0.5, 0),
    TagId1(0.5,-0.5,0),
=======
  private double m_driveSpeed = 0.15;  //constant
  private double m_speed = 0.15; //changes
  private double m_xOffset, m_zOffset; //offsets from april tag

>>>>>>> Stashed changes
  private Pose2d m_goalPose = new Pose2d(); //instantiates pose of where robot is
  private double m_vectorDistance; //Vector distance from robot to April tag
  public enum TagNumber{ //method of constants for april tag offsets
    TagId1(1,-1,0),
<<<<<<< Updated upstream
=======
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
  public enum Position{ //variables for 2nd constructor, what the offset is
=======
<<<<<<< HEAD
  //enum to say what position we want to go to -- what offset
  public enum Position{
=======
  public enum Position{ //variables for 2nd constructor, what the offset is
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
  //DriveSubsystem.get().tankDrive(-0.25, -0.25);
    double z = -AprilTagSubsystem.get().getDistance();
    double x = AprilTagSubsystem.get().getX();
=======
<<<<<<< HEAD

  //rotation math to translate the apriltag data + offsets into a field location in the odometry frame
  //get the pose we intend to reach by the end of the command 
  double z = AprilTagSubsystem.get().getDistance() + m_zOffset;
  double x = AprilTagSubsystem.get().getX() + m_xOffset;
  double aprilTagYaw = AprilTagSubsystem.get().getYaw();
  Pose2d currPose = DriveSubsystem.get().getPose();
  double currHeading = DriveSubsystem.get().getHeading();
  currHeading = currHeading < 0 ? currHeading +360 : currHeading;
  double rotationToField = Math.toRadians(-aprilTagYaw) - currPose.getRotation().getRadians();
  Translation2d goalTranslation = (new Translation2d(-z,-x)).rotateBy(new Rotation2d(rotationToField));
  m_goalPose = new Pose2d(currPose.getX() + goalTranslation.getX(), currPose.getY() - goalTranslation.getY(), currPose.getRotation());
=======
  //DriveSubsystem.get().tankDrive(-0.25, -0.25);
    double z = -AprilTagSubsystem.get().getDistance();
    double x = AprilTagSubsystem.get().getX();
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes

    Pose2d currPose = DriveSubsystem.get().getPose(); //current pose based on navx data
    m_goalPose = currPose.transformBy(new Transform2d(new Translation2d(z+m_zOffset, x+m_xOffset), new Rotation2d()));
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
    SmartDashboard.putNumber("Goal X", m_goalPose.getX());
    SmartDashboard.putNumber("Goal Y", m_goalPose.getY());
    SmartDashboard.putNumber("Curr X", DriveSubsystem.get().getPose().getX());
    SmartDashboard.putNumber("Curr Y", DriveSubsystem.get().getPose().getY());
   
    //drive based on turns computed from our current position and our goal position
=======
>>>>>>> Stashed changes
    Pose2d currPose = DriveSubsystem.get().getPose();

    double z = -AprilTagSubsystem.get().getDistance(); //distance out of april tag is negative, fix that
    double x = AprilTagSubsystem.get().getX();

    if(z != 0){
      //m_goalPose = currPose.transformBy(new Transform2d(new Translation2d(z+m_zOffset, x+m_xOffset), new Rotation2d()));
    }

>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
    DriveSubsystem.get().arcadeDrive(m_driveSpeed, getTurn(m_goalPose, DriveSubsystem.get().getPose()));
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< Updated upstream
    DriveSubsystem.get().tankDrive(0, 0); //stop robot
=======
<<<<<<< HEAD
    //stop the drive train when we're done
    DriveSubsystem.get().tankDrive(0, 0);
=======
    DriveSubsystem.get().tankDrive(0, 0); //stop robot
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2
>>>>>>> Stashed changes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
    if(Math.abs(m_goalPose.getX() - DriveSubsystem.get().getPose().getX()) < 0.1) { //if we've reached the goal
      return true;
    }
    return false;
  }
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2

    //end when we reach our goal point
    return Math.abs(m_goalPose.getX() - DriveSubsystem.get().getPose().getX()) < 0.1 && Math.abs(m_goalPose.getY() - DriveSubsystem.get().getPose().getY()) < 0.1;
  
  }
  private double getTurn(Pose2d goalPoint, Pose2d currPoint) {

    SmartDashboard.putNumber("Goal X", goalPoint.getX());
    SmartDashboard.putNumber("Goal Y", goalPoint.getY());
    SmartDashboard.putNumber("Curr X", currPoint.getX());
    SmartDashboard.putNumber("Curr Y", currPoint.getY());

    // just the distance formula
    double xDifference = goalPoint.getX() - currPoint.getX();
    double yDifference = goalPoint.getY() - currPoint.getY();
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
    double distanceSquared = xDifference*xDifference + yDifference * yDifference;

    //get the angle of the vector between our current robot location and our goal point in the odometry frame
    //and adjust it to be in the robot frame
    double vectorAngle = Math.atan2(yDifference,xDifference);
    double robotAngle = DriveSubsystem.get().getHeading();
    robotAngle = robotAngle < 0 ? robotAngle +360 : robotAngle;
    double angleChangeRadians = Math.toRadians(robotAngle);
    double vectorAngleAdjusted = vectorAngle - angleChangeRadians;
=======
>>>>>>> Stashed changes
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
>>>>>>> 1e66c1128d5ea986ba83a0c68f5fa5fd243783b2

    //compute drive speed and turn angle using the angle, 
    //half speed if we are less than half a meter from our goal point
    //turn speed is in relation to the sine of the computed angle
    //since the y-axis of the robot frame is parallel to the wheel axle
    //this means that our turn speed is the greatest when the goal point is directly next to us
    //and least when the goal point is directly in front of us
    //drive speed is in relation to the cosine of the computed angle
    //this means that our speed is the greatest when the goal point is directly in front of us
    //and least when the goal point is directly next to us
    double speedLimitFactor = (distanceSquared > 0.25 ? 1 : 0.5);
    m_driveSpeed = m_speed*Math.cos(vectorAngleAdjusted) * speedLimitFactor;
    return Math.sin(vectorAngleAdjusted)*m_speed * speedLimitFactor;
  }
}
