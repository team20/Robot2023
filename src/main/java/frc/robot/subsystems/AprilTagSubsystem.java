// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
  NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("apriltag-poses");
  private static AprilTagSubsystem s_subsystem;
  private double[][] poses = new double[8][3];
  /** Creates a new ApriltagSubsystem. */
  public AprilTagSubsystem() {
    s_subsystem = this;
  }

  public static AprilTagSubsystem get(){
    return s_subsystem;
  }
  @Override
  public void periodic() {
    double[] defaultValue = {-1,-1,-1};
    for(int i = 7; i<8; ++i){
      //if(){
        poses[i] = m_aprilTagTable.getEntry("tagid" + (i+1)).getDoubleArray(defaultValue);
        System.out.println(Arrays.toString(poses[i]));
      //}
    }
    // This method will be called once per scheduler run
  }
}
