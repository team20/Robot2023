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

public class EasyTagAlignCommand extends CommandBase{
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
    public EasyTagAlignCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(DriveSubsystem.get());
    }
    public EasyTagAlignCommand(double xOffset, double zOffset) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_xOffset = xOffset;
        m_zOffset = zOffset;
        addRequirements(DriveSubsystem.get());
    }
    public EasyTagAlignCommand(TagNumber tagId, Position pos, double zOffset) {
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

    @Override
    public void initialize() {
        double z = -AprilTagSubsystem.get().getDistance();
        double x = AprilTagSubsystem.get().getX();
        double yaw = AprilTagSubsystem.get().getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double z = -AprilTagSubsystem.get().getDistance(); //distance out of april tag is negative, fix that
        double x = AprilTagSubsystem.get().getX();

        double 
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

    private double getTurn() {
        
    }

}
