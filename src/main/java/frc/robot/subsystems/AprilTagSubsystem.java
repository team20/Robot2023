// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

//test commit
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
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
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    // read values periodically
    double x = tx.getDouble(-1.0);
    double y = ty.getDouble(-1.0);
    double area = ta.getDouble(-1.0);
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    // This method will be called once per scheduler run
  }
}
