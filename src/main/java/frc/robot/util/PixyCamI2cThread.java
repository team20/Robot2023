// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants.ArduinoConstants;

/** Add your docs here. */
public class PixyCamI2cThread implements Runnable{

    private PixyCamObjectMap m_map;
    private I2C m_pixyCamDevice = new I2C(Port.kMXP, ArduinoConstants.kPixyCamAddress);

    public PixyCamI2cThread(PixyCamObjectMap map){
        m_map = map;
    }
    public void run(){
        //4 chunks * 12 bytes of data per chunk
        int numBytes = 4 * 12;
        ByteBuffer buffer = ByteBuffer.allocate(numBytes);
        m_pixyCamDevice.read(ArduinoConstants.kPixyCamAddress, numBytes, buffer);

        interpretBuffer(buffer);
    }

    public void interpretBuffer(ByteBuffer buffer){
		byte b;
		int counter = 0;
		int signature = 0;
		int x = 0;
		int y = 0;
		int width = 0;
		int height = 0;
		int index = 0;
		int age = 0;

		//Read all bytes from buffer
		for(int i = 0; i<buffer.capacity(); ++i){
			b = buffer.get();
			//12byte block
			//2byte signature
			//2byte x location
			//2byte y location
			//2byte height of object
			//2byte width of object
			//1byte tracking index
			//1byte age of object
			//when we've read in a whole object from the stream, add the object to our list of tracked objects
			switch(counter%12){
				case 0:
					signature = b & 0xFF;
					break;
				case 1:
					signature = (signature & 0xFF) << 8 | (b & 0xFF);
					break;
				case 2:
					x = b & 0xFF;
					break;
				case 3:
					x = (x & 0xFF) << 8 | (b & 0xFF);
					break;
				case 4:
					y = b & 0xFF;
					break;
				case 5:
					y = (y & 0xFF) << 8 | (b & 0xFF);
					break;
				case 6:
					width = b & 0xFF;
					break;
				case 7:
					width = (width & 0xFF) << 8 | (b & 0xFF);
					break;
				case 8:
					height = b & 0xFF;
					break;
				case 9:
					height = (height & 0xFF) << 8 | (b & 0xFF);
					break;
				case 10:
					index = b & 0xFF;
					break;
				case 11:
					age = b & 0xFF;
					try{
						m_map.get(index).set(signature, x, y, width, height, index, age);
					}catch(Exception e){
						m_map.set(index, new PixyCamObject(signature, x, y, width, height, index, age));
					}
					break;
			}
			++counter;
		}
	}
}
