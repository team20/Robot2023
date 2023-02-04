// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

//test commit
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
  public double m_x, m_y, m_z, m_pitch, m_yaw, m_roll;

  NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
  private static AprilTagSubsystem s_subsystem;
  private ArrayList<Double> rollingAverageYaw = new ArrayList<Double>();
  private double rollingAverageValue = 0;
  /** Creates a new ApriltagSubsystem. */
  public AprilTagSubsystem() {
    s_subsystem = this;
  }

  public static AprilTagSubsystem get() {
    return s_subsystem;
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry camtran = table.getEntry("campose");
    NetworkTableEntry tx = table.getEntry("tx");

    // read values periodically
    double[] translation = camtran.getDoubleArray(new double[6]);

    if(!translation.equals(new double[6])){
      m_x = translation[0];
      m_y = translation[1];
      m_z = translation[2];
      m_pitch = translation[3];
      m_yaw = translation[4]; // [-71, 66]
      m_roll = translation[5];
    }
    //m_yaw = tx.getDouble(0);
    rollingAverageYaw.add(m_yaw);
    if(rollingAverageYaw.size() > 10){
      double removeValue = rollingAverageYaw.remove(0);
      rollingAverageValue -= removeValue/rollingAverageYaw.size();
    }
    rollingAverageValue += m_yaw/rollingAverageYaw.size();
    m_yaw = tx.getDouble(0);
    double angle = Math.toDegrees(Math.atan(m_x/m_z));
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Angle",angle);
    SmartDashboard.putNumber("LimelightX", m_x);
    SmartDashboard.putNumber("LimelightY", m_y);
    SmartDashboard.putNumber("LimelightZ", m_z);
    SmartDashboard.putNumber("LimelightPitch", m_pitch);
    SmartDashboard.putNumber("LimelightYaw", m_yaw);
    SmartDashboard.putNumber("LimelightRoll", m_roll);
    // This method will be called once per scheduler run

  }
  public double getDistance(){
    return m_z;
  }
  public double getYaw(){
    return m_yaw;
  }
  public double getX(){
    return m_x;
  }
}
