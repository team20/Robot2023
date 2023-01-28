// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

//test commit
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
  private double m_x, m_y, m_z, m_pitch, m_yaw, m_roll;

  NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
  private static AprilTagSubsystem s_subsystem;

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
    // read values periodically
    double[] translation = camtran.getDoubleArray(new double[6]);
    m_x = translation[0];
    m_y = translation[1];
    m_z = translation[2];
    m_pitch = translation[3];
    m_yaw = translation[4];
    m_roll = translation[5];

    // post to smart dashboard periodically
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
}
