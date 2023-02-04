// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
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
  private double m_speed = 0.25;
  private double m_xOffset, m_zOffset;

  public enum TagNumber{
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

    private TagNumber(double offsetLeft, double offsetRight, double offsetMiddle){
      yOffsetLeft = offsetLeft;
      yOffsetRight = offsetRight;
      yOffsetMiddle = offsetMiddle;
    }
  }

  public enum Position{
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
    switch(pos){
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

  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //DriveSubsystem.get().tankDrive(-0.25, -0.25);
    

    DriveSubsystem.get().arcadeDrive(-m_speed, getTurn());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return AprilTagSubsystem.get().getDistance() > -0.6;
  }

  private double getTurn() {

    //z is out of the apriltag(will always return a negative value)
    //x is left right on the apriltag
    double z = -AprilTagSubsystem.get().getDistance() + m_zOffset;
    double x = AprilTagSubsystem.get().getX() + m_xOffset;
    double angle = Math.atan(x/z);
    
    if(AprilTagSubsystem.get().getDistance()<0){
      m_speed = 0.25;
      System.out.println( Math.toDegrees(angle) + " "  + Math.cos(angle)*m_speed);
      return Math.cos(angle)*m_speed;
    }else{
      m_speed = 0;
      return 0.1*Math.signum(Math.sin(angle));
    }
  }
}
