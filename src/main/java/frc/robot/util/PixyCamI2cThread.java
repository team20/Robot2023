// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.ArduinoConstants;

/** Add your docs here. */
public class PixyCamI2cThread implements Runnable {
	private PixyCamObjectMap m_map;
	private I2C m_pixyCamDevice;

	public PixyCamI2cThread(I2C pixyCamDevice, PixyCamObjectMap map) {
		m_pixyCamDevice = pixyCamDevice;
		m_map = map;
	}

	public void run() {
		while (true) {
			// System.out.println("RUNNING THREAD");
			// 4 chunks * 9 bytes of data per chunk
			System.out.println("ATTEMPTING WRITE");
			int numBytes = 4 * 9;
			byte[] buffer = new byte[numBytes];
			// m_pixyCamDevice.writeBulk(new byte[1]);
			boolean status = m_pixyCamDevice.read(ArduinoConstants.kPixyCamAddress, numBytes, buffer);
			if (status) {
				System.out.println("Unsuccessful");
			} else {
				System.out.println("Successful");
				interpretBuffer(buffer);
			}
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
		}
	}

	public void interpretBuffer(byte[] buffer) {
		byte b;
		int counter = 0;
		int signature = 0;
		int x = 0;
		int y = 0;
		int width = 0;
		int height = 0;
		int index = 0;
		int age = 0;

		// Read all bytes from buffer
		for (int i = 0; i < buffer.length; ++i) {
			b = buffer[i];
			System.out.println(b);
			// 9 byte block
			// 1 byte signature
			// 2 byte x location
			// 1 byte y location
			// 2 byte width of object
			// 1 byte height of object
			// 1 byte tracking index
			// 1 byte age of object
			// when we've read in a whole object from the stream, add the object to our list
			// of tracked objects
			switch (counter % 9) {
				case 0:
					// System.out.println(b & 0xFF);
					signature = (~b & 0xFF);
					break;
				case 1:
					x = b & 0xFF;
					break;
				case 2:
					x = (x & 0xFF) << 8 | (b & 0xFF);
					break;
				case 3:
					y = b & 0xFF;
					break;
				case 4:
					width = b & 0xFF;
					break;
				case 5:
					width = (width & 0xFF) << 8 | (b & 0xFF);
					break;
				case 6:
					height = b & 0xFF;
					break;
				case 7:
					index = b & 0xFF;
					break;
				case 8:
					age = b & 0xFF;
					// System.out.println(signature);
					if (signature != 255) {
						try {
							m_map.get(index).set(signature, x, y, width, height, index, age);
						} catch (Exception e) {
							m_map.set(index, new PixyCamObject(signature, x, y, width, height, index, age));
						}
					}
					break;
			}
			++counter;
		}
	}
}
