// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
	/**
	 * The I2C device we're connecting to. Port.kMXP means we use the I2C connection
	 * on the MXP port, which runs through the navX
	 */
	private I2C i2c = new I2C(Port.kMXP, 0x18);

	/** The byte that indicates what LED mode we want to use */
	private byte[] m_statusCode = new byte[1];

	/** The bytes that control the LED mode */
	private enum StatusCode {
		DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR((byte) 9),
		ORANGE_THEATER_LIGHTS((byte) 10),
		BLUE_THEATER_LIGHTS((byte) 11),
		RED_THEATER_LIGHTS((byte) 12),
		MOVING_GREEN_AND_BLUE_GRADIENT((byte) 13),
		GREEN_THEATER_LIGHTS((byte) 14),
		MOVING_RED_AND_GREEN_GRADIENT((byte) 15),
		BLUE_BACK_AND_FORTH_TIMER((byte) 16), // TODO: timer currently has a bug (look in arduino IDE)
		GREEN_BACK_AND_FORTH_TIMER((byte) 17),
		PURPLE_BLINKING((byte) 18),
		YELLOW_BLINKING((byte) 19);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		setCode(StatusCode.DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR);
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		i2c.writeBulk(m_statusCode);
	}

	public void setCode(StatusCode code) {
		m_statusCode[0] = code.code;
	}
}
