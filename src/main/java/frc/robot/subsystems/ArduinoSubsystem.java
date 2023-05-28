// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArduinoConstants;

public class ArduinoSubsystem extends SubsystemBase {
	private static ArduinoSubsystem s_subsystem;
	/**
	 * The I2C device we're connecting to. Port.kMXP means we use the I2C connection
	 * on the MXP port, which runs through the navX
	 */
	private I2C i2c = new I2C(I2C.Port.kMXP, ArduinoConstants.kAddress);
	// private SerialPort m_usbPort = new SerialPort(250000, SerialPort.Port.kUSB);
	/** The byte that indicates what LED mode we want to use */
	private byte[] m_statusCode = new byte[1];

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 8),
		BLINKING_YELLOW((byte) 9),
		BLINKING_PURPLE((byte) 10),
		MOVING_GREEN_AND_RED_GRADIENT((byte) 11),
		MOVING_GREEN_AND_BLUE_GRADIENT((byte) 12),
		RAINBOW_PARTY_FUN_TIME((byte) 16),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

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
		setCode(StatusCode.BLINKING_YELLOW);
		// m_usbPort.reset();
	}

	public static ArduinoSubsystem get() {
		return s_subsystem;
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		byte[] e = new byte[30];
		boolean shouldWrite = true;
		if (shouldWrite) {
			i2c.writeBulk(e);
		} else {
			i2c.read(0x18, 30, e);
		}
		System.out.println((int) e[0]);
	}

	public void setCode(StatusCode code) {
		m_statusCode[0] = code.code;
	}
}
