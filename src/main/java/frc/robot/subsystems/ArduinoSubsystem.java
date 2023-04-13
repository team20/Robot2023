// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArduinoConstants;
import frc.robot.util.PixyCamI2cThread;
import frc.robot.util.PixyCamObject;
import frc.robot.util.PixyCamObjectMap;

public class ArduinoSubsystem extends SubsystemBase {
	private static ArduinoSubsystem s_subsystem;

	/**
	 * The I2C device we're connecting to. Port.kMXP means we use the I2C connection
	 * on the MXP port, which runs through the navX
	 */
	private I2C m_ledDevice = new I2C(Port.kMXP, ArduinoConstants.kLEDAddress);
    private I2C m_pixyCamDevice = new I2C(Port.kMXP, ArduinoConstants.kPixyCamAddress);
	/** The byte that indicates what LED mode we want to use */
	private byte[] m_statusCode = new byte[1];

	private ByteBuffer m_readBuffer;
	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 8),
		BLINKING_YELLOW((byte) 9),
		BLINKING_PURPLE((byte) 10),
		MOVING_GREEN_AND_RED_GRADIENT((byte) 11),
		MOVING_GREEN_AND_BLUE_GRADIENT((byte) 12),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}


	

	PixyCamObjectMap m_detectedObjects = new PixyCamObjectMap();
	private Thread m_pixyCamThread;
	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Arduino subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
		setCode(StatusCode.DEFAULT);

		m_pixyCamThread = new Thread(new PixyCamI2cThread(m_pixyCamDevice, m_detectedObjects));
		m_pixyCamThread.setDaemon(true);
		m_pixyCamThread.start();
	}

	public static ArduinoSubsystem get() {
		return s_subsystem;
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		//if(!m_pixyCamThread.isAlive()){
			
		//}
		Instant now= Instant.now();
		SmartDashboard.putNumber("Largest Cube X", getLargestCube().get()[1]);
		// System.out.println(Duration.between(now, Instant.now()).toMillis());
		//m_ledDevice.writeBulk(m_statusCode);
	}

	public void setCode(StatusCode code) {
		m_statusCode[0] = code.code;
	}

	public PixyCamObject getLargestCube(){
		PixyCamObject[] arr = m_detectedObjects.getArray();
		PixyCamObject largest = new PixyCamObject(0, 0, 0, 0, 0, 0, 0);
		for(int i = 0; i < arr.length; ++i){
			PixyCamObject currObject = arr[i];
			if(currObject!= null && currObject.isExpired()){
				m_detectedObjects.set(i, null);
			}
			if(currObject != null && !currObject.isExpired() && currObject.get()[0] == 1 && currObject.get()[3] > largest.get()[3]){
				largest = currObject;
			}
		}
		return largest;
	}
}
