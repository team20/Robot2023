// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
	private static ArduinoSubsystem s_subsystem;
	/**
	 * The I2C device we're connecting to. Port.kMXP means we use the I2C connection
	 * on the MXP port, which runs through the navX
	 */
	private final SerialPort usb = new SerialPort(9600, Port.kUSB);

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 8),
		BLINKING_YELLOW((byte) 9),
		BLINKING_PURPLE((byte) 10),
		MOVING_GREEN_AND_RED_GRADIENT((byte) 11),
		MOVING_GREEN_AND_BLUE_GRADIENT((byte) 12),
		RAINBOW_PARTY_FUN_TIME((byte) 16),
		SUPER_RAINBOW_PARTY_FUN_TIME((byte) 17++++),
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
	}

	public static ArduinoSubsystem get() {
		return s_subsystem;
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
	}

	public void setCode(StatusCode code) {
		usb.write(new byte[] { code.code }, 1);
	}

	public Command writeStatus(StatusCode code) {
		return runOnce(() -> setCode(code));
	}
}
