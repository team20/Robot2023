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

/**
 * Operates the Robot's ability to detect and identify AprilTags using LimeLight
 * cameras.
 * 
 * @author Jonathan Waters
 * @author Andrew Hwang
 * @author Jamis Orr
 */
public class AprilTagSubsystem extends SubsystemBase {
  /**
   * The x translation from the Limelight to the AprilTag.
   */
  protected double m_x;
  /**
   * The y translation from the Limelight to the AprilTag.
   */
  protected double m_y;
  /**
   * The z translation from the Limelight to the AprilTag.
   */
  public double m_z;
  /**
   * The pitch angle (angle along the x-axis) between the LimeLight and AprilTag.
   */
  protected double m_pitch;
  /**
   * The yaw angle (angle along the y-axis) between the LimeLight and AprilTag.
   */
  protected double m_yaw;
  /**
   * The roll angle (angle between the z-axis) between the LimeLight and AprilTag.
   */
  protected double m_roll;

  /**
   * Instantiates the {@code NetworkTable} in {@code AprilTagSubsystem}.
   */
  NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
  private static AprilTagSubsystem s_subsystem;

  private MedianFilter m_filterX = new MedianFilter(10);
  private MedianFilter m_filterY = new MedianFilter(10);
  private MedianFilter m_filterZ = new MedianFilter(10);
  private MedianFilter m_filterPitch = new MedianFilter(10);
  private MedianFilter m_filterRoll = new MedianFilter(10);
  private MedianFilter m_filterYaw = new MedianFilter(10);

  private boolean m_tagInView;

  /** Creates a new ApriltagSubsystem. */
  public AprilTagSubsystem() { // constructor, makes the apriltagSubsystem = to the first instance called
    s_subsystem = this;
  }

  public static AprilTagSubsystem get() { // returns the first instance called, guarantees that there's not conflicting
                                          // instances
    return s_subsystem;
  }

  /**
   * A method run periodically (every 20 ms).
   */
  @Override
  public void periodic() {
    /**
     * Gets 3D transform of the camera given that the frame of reference is the position of the April Tag.
     */
    NetworkTableEntry camtran = m_aprilTagTable.getEntry("camerapose_targetspace"); 
    
    /**
     * Gets the array of camerapose_targetspace, which includes x, y, z, pitch, yaw, and roll respectively
     */
    double[] translation = camtran.getDoubleArray(new double[6]); 

    // median filters for all values, makes data more steady even w/ camera-misread outliers
    if (!translation.equals(new double[6])) { 
      m_x = m_filterX.calculate(translation[0] - .155); // subtract offset of limelight from center of camera
      m_y = m_filterY.calculate(translation[1]);
      m_z = m_filterZ.calculate(translation[2]);
      m_pitch = m_filterPitch.calculate(translation[3]);
      m_yaw = m_filterYaw.calculate(translation[4]); // [-71, 66]
      m_roll = m_filterRoll.calculate(translation[5]);
      m_tagInView = true;
    } else {
      m_tagInView = false;
    }

    // Posts the translation values into SmartDashboard
    SmartDashboard.putNumber("LimelightX", m_x);
    SmartDashboard.putNumber("LimelightY", m_y);
    SmartDashboard.putNumber("LimelightZ", m_z);
    SmartDashboard.putNumber("LimelightPitch", m_pitch);
    SmartDashboard.putNumber("LimelightYaw", m_yaw);
    SmartDashboard.putNumber("LimelightRoll", m_roll);
    SmartDashboard.putBoolean("Tag in View", m_tagInView);
  }

  /**
   * Returns the distance between the camera and the AprilTag
   * 
   * @return the distance between the camera and the AprilTag
   */
  public double getDistance() {
    return m_z;
  }

  /**
   * Returns the yaw angle between the camera and the AprilTag
   * 
   * @return the yaw angle between the camera and the AprilTag
   */
  public double getYaw() {
    return m_yaw;
  }

  /**
   * Returns the x (horizontal) translation from the AprilTag
   * 
   * @return the x (horizontal) translation from the AprilTag
   */
  public double getX() {
    return m_x;
  }

  /**
   * Returns a boolean statement if the AprilTag is within the view of the camera.
   * 
   * @return a boolean statement if the AprilTag is within the view of the camera.
   */
  public boolean tagInView() {
    return m_tagInView;
  }
}
