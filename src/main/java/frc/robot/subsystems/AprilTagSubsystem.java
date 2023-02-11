// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
//test commit
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
  public double m_x, m_y, m_z, m_pitch, m_yaw, m_roll; //public variables for limelight data

  NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight"); //instantiate network
  private static AprilTagSubsystem s_subsystem; 


  private MedianFilter m_filterX = new MedianFilter(10); 
  private MedianFilter m_filterY = new MedianFilter(10);
  private MedianFilter m_filterZ = new MedianFilter(10);
  private MedianFilter m_filterPitch = new MedianFilter(10);
  private MedianFilter m_filterRoll = new MedianFilter(10);
  private MedianFilter m_filterYaw = new MedianFilter(10);

private boolean m_tagInView;
  /** Creates a new ApriltagSubsystem. */
  public AprilTagSubsystem() { //constructor, makes the apriltagSubsystem = to the first instance called
    s_subsystem = this;
  }

  public static AprilTagSubsystem get() { //returns the first instance called, guarantees that there's not conflicting instances
    return s_subsystem;
  }

  @Override
  public void periodic() { //called every 20 ms
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //gets networktable from our limelight, using key we set up
    NetworkTableEntry camtran = table.getEntry("camerapose_targetspace"); //Gets "3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))"
    // read values periodically
    double[] translation = camtran.getDoubleArray(new double[6]); //gets array of x, y, z, pitch, yaw, roll from camtran

    if(!translation.equals(new double[6])){ //median filters for all values, makes data more steady even w/ camera-misread outliers
      m_x = m_filterX.calculate(translation[0] - .155); //subtract offset of limelight from center of camera
      m_y = m_filterY.calculate(translation[1]);
      m_z = m_filterZ.calculate(translation[2]);
      m_pitch = m_filterPitch.calculate(translation[3]);
      m_yaw = m_filterYaw.calculate(translation[4]); // [-71, 66]
      m_roll = m_filterRoll.calculate(translation[5]);
      m_tagInView = true;
    }else{
      m_tagInView = false;
    }
    

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", m_x);
    SmartDashboard.putNumber("LimelightY", m_y);
    SmartDashboard.putNumber("LimelightZ", m_z);
    SmartDashboard.putNumber("LimelightPitch", m_pitch);
    SmartDashboard.putNumber("LimelightYaw", m_yaw);
    SmartDashboard.putNumber("LimelightRoll", m_roll);
    SmartDashboard.putBoolean("Tag in View", m_tagInView);
    // This method will be called once per scheduler run

  }
  public double getDistance(){ //return distance away from april tag
    return m_z;
  }
  public double getYaw(){ //gets rotation from april tag 
    return m_yaw;
  }
  public double getX(){ //gets horizontal distance from apriltag
    return m_x;
  }
  public boolean tagInView(){ //gets if tag is in view
    return m_tagInView;
  }
}
