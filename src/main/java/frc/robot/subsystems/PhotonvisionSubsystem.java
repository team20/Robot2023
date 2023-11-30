// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 *  Operates the Robot's ability to detect AprilTags using Photonvision
 *  @author Andrew Hwang
 */
public class PhotonvisionSubsystem extends SubsystemBase {

    // Member Variables
	public double m_x, m_y, m_z, m_pitch, m_yaw, m_qw, m_qx, m_qy, m_qz;
    NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private static PhotonvisionSubsystem s_subsystem;
    public boolean m_tagInView;


    // Median Filters
    private MedianFilter m_filterX, m_filterY, m_filterZ, m_filterPitch, m_filterYaw, 
    m_filterQW, m_filterQX, m_filterQY, m_filterQZ = new MedianFilter(10);

    // Constructor
    public PhotonvisionSubsystem() {
        // Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Photonvision subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
	}

    public static PhotonvisionSubsystem get() {
        return s_subsystem;
    }

    /**
     * A method run twenty times per second.
     */
    @Override
    public void periodic() {
        // targetPose contains 7 values
        // The first three are x, y, and z values of the target relative to the robot
        // The last four values are represented by a quaternion: qw, qx, qy, qz
        // qw is a scalar multplied to a three tuple vector [qx, qy, qz]
        NetworkTableEntry robottran = photonTable.getEntry("targetPose");

        // targetPitch and targetYaw receives the pitch and the yaw of the robot relative to the tag
        NetworkTableEntry pitchEntry = photonTable.getEntry("targetPitch");
        NetworkTableEntry yawEntry = photonTable.getEntry("targetYaw");

        double[] targetRelative = robottran.getDoubleArray(new double[6]);

        // If a tag is in view of the camera and is recording data onto the Network Tables
        if (!targetRelative.equals(new double[6])) {
            m_x = m_filterX.calculate(targetRelative[0]);
            m_y = m_filterY.calculate(targetRelative[1]);
            m_z = m_filterZ.calculate(targetRelative[2]);
            m_qw = m_filterQW.calculate(targetRelative[3]);
            m_qx = m_filterQX.calculate(targetRelative[4]);
            m_qy = m_filterQY.calculate(targetRelative[5]);
            m_qz = m_filterQZ.calculate(targetRelative[6]);
            m_pitch = m_filterPitch.calculate(pitchEntry.getDouble(0));
            m_yaw = m_filterYaw.calculate(yawEntry.getDouble(0));
            m_tagInView = true;
        } else {
            m_tagInView = false;
        }

        // Data is updated onto SmartDashboard
        SmartDashboard.putNumber("Photonvision X", m_x);
		SmartDashboard.putNumber("Photonvision Y", m_y);
		SmartDashboard.putNumber("Photonvision Z", m_z);
        SmartDashboard.putNumber("Photonvision QW", m_qw);
        SmartDashboard.putNumber("Photonvision QX", m_qx);
		SmartDashboard.putNumber("Photonvision QY", m_qy);
		SmartDashboard.putNumber("Photonvision QZ", m_qz);
		SmartDashboard.putNumber("Photonvision Pitch", m_pitch);
		SmartDashboard.putNumber("Photonvision Yaw", m_yaw);
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
