#include <Pixy2.h>
#include <Wire.h>
// Initialize global pixy object
Pixy2 pixy;

void setup() {
	pixy.init();
	Wire.begin(0x20);
}
void loop() {
	// Start a transmission to the navX
	Wire.beginTransmission(0x32);
	// Get the tracked objects
	pixy.ccc.getBlocks();
	// Select the largest object and store its data into an int array
	byte objectData[8];
	objectData[0] = (byte)((pixy.ccc.blocks[0].m_x >> 8) & 0xFF);
	objectData[1] = (byte)(pixy.ccc.blocks[0].m_x & 0xFF);
	objectData[2] = (byte)pixy.ccc.blocks[0].m_y;
	objectData[3] = (byte)((pixy.ccc.blocks[0].m_width >> 8) & 0xFF);
	objectData[4] = (byte)(pixy.ccc.blocks[0].m_width & 0xFF);
	objectData[5] = (byte)pixy.ccc.blocks[0].m_height;
	objectData[6] = (byte)pixy.ccc.blocks[0].m_signature;
	Wire.write(objectData, 8);
	Wire.endTransmission();
	delay(40);
}