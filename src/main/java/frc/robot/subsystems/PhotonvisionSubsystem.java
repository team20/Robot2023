// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 *  Operates the Robot's ability to detect AprilTags using Photonvision
 *  @author Andrew Hwang
 */
public class PhotonvisionSubsystem extends SubsystemBase {

    // Member Variables
	public double m_latencyMillis;
    public double[] m_targetPose;
    NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("photonvision/MAIN");
    private static PhotonvisionSubsystem s_subsystem;

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
        m_latencyMillis = photonTable.getEntry("latencyMillis").getDouble(0.0);
        m_targetPose = photonTable.getEntry("targetPose").getDoubleArray(new double[7]);
        SmartDashboard.putNumber("latencyMillis", m_latencyMillis);
		SmartDashboard.putNumberArray("targetPose", m_targetPose);
    }
    /**
	 * Returns the distance between the camera and the AprilTag
	 * 
	 * @return the distance between the camera and the AprilTag
	 */
	public double getLatencyMillis() {
		return m_latencyMillis;
	}

	/**
	 * Returns the yaw angle between the camera and the AprilTag
	 * 
	 * @return the yaw angle between the camera and the AprilTag
	 */
	public double[] getTargetPose() {
		return m_targetPose;
	}

}
