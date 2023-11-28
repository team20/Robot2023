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
 * Operates the Robot's ability to detect AprilTags using Photonvision
 */
public class PhotonvisionSubsystem extends SubsystemBase{

    // Member Variables
	public double m_x, m_y, m_z, m_pitch, m_yaw, m_qw, m_qx, m_qy, m_qz;
    NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    private static PhotonvisionSubsystem s_subsystem;
    public boolean m_tagInView;


    // Median Filters
    private MedianFilter m_filterX = new MedianFilter(10);
	private MedianFilter m_filterY = new MedianFilter(10);
	private MedianFilter m_filterZ = new MedianFilter(10);
	private MedianFilter m_filterPitch = new MedianFilter(10);
	private MedianFilter m_filterYaw = new MedianFilter(10);
	private MedianFilter m_filterQW = new MedianFilter(10);
	private MedianFilter m_filterQX = new MedianFilter(10);
	private MedianFilter m_filterQY = new MedianFilter(10);
	private MedianFilter m_filterQZ = new MedianFilter(10);
	private MedianFilter m_filterRollT = new MedianFilter(10);
	private MedianFilter m_filterYawT = new MedianFilter(10);


    public PhotonvisionSubsystem() { // constructor, makes the apriltagSubsystem = to the first instance called
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


    @Override
    public void periodic() {
        // targetpose contains 7 values
        // The first three are x, y, and z values of the target relative to the robot
        // The last four values are represented by a quaternion: qw, qx, qy, qz
        // qw is a scalar multplied to a three tuple vector [qx, qy, qz]
        NetworkTableEntry robottran = photonTable.getEntry("targetPose");
        NetworkTableEntry pitchEntry = photonTable.getEntry("targetPitch");
        NetworkTableEntry yawEntry = photonTable.getEntry("targetYaw");

        double[] targetRelative = robottran.getDoubleArray(new double[6]);
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

        SmartDashboard.putNumber("Photonvision X", m_x);
		SmartDashboard.putNumber("Photonvision Y", m_y);
		SmartDashboard.putNumber("Photonvision Z", m_z);
		SmartDashboard.putNumber("Photonvision Pitch", m_pitch);
		SmartDashboard.putNumber("Photonvision Yaw", m_yaw);
		SmartDashboard.putBoolean("Tag in View", m_tagInView);
        SmartDashboard.putNumber("Photonvision QW", m_qw);
        SmartDashboard.putNumber("Photonvision QX", m_qx);
		SmartDashboard.putNumber("Photonvision QY", m_qy);
		SmartDashboard.putNumber("Photonvision QZ", m_qz);

    }
    // TODO: add getter methods
}
