#include <Pixy2.h>
#include <Wire.h>
// Initialize global pixy object
// When uploading into arduino go to Tools --> Processer --> and make sure it is ATmega328P (Old Bootloader)  (also I am unsure if this affects it but the programmer is also Arduino as ISP (ATmega32U4))
Pixy2 pixy;

void setup() {
	pixy.init();
	Wire.begin(20);
  Serial.begin(9600);
}
void loop() {
	// Start a transmission to the navX
	Wire.beginTransmission(0x32);
  Serial.println("Signature");
	// Get the tracked objects
	pixy.ccc.getBlocks();
	// Select the largest object and store its data into an int array
	byte objectData[7];
	objectData[0] = (byte)((pixy.ccc.blocks[0].m_x >> 8) & 0xFF);
	objectData[1] = (byte)(pixy.ccc.blocks[0].m_x & 0xFF);
	objectData[2] = (byte)pixy.ccc.blocks[0].m_y;
	objectData[3] = (byte)((pixy.ccc.blocks[0].m_width >> 8) & 0xFF);
	objectData[4] = (byte)(pixy.ccc.blocks[0].m_width & 0xFF);
	objectData[5] = (byte)pixy.ccc.blocks[0].m_height;
	objectData[6] = (byte)pixy.ccc.blocks[0].m_signature;
	Wire.write(objectData, 7);
  Serial.println("Signature:" + objectData[0]);
	// Wire.endTransmission();
	delay(40);
}


// #include <Wire.h>
// #include <Pixy2.h>



// Pixy2 pixy;

// uint8_t delimiter[4] = {'a','b','c','d'};

// // setup() function -- runs once at startup --------------------------------
// void setup() {
//   Wire.begin(0x30);         //begin I2C
// }
// void loop() {

//   pixy.ccc.getBlocks();
  
//   if(pixy.ccc.numBlocks){
//     for (int i=0; i<pixy.ccc.numBlocks; ++i){
//       Wire.beginTransmission(0x32);
//       Wire.write(delimiter, 4);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_signature, 2);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_x, 2);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_y, 2);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_width, 2);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_height, 2);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_index, 1);
//       Wire.write((uint8_t*) &pixy.ccc.blocks[i].m_age, 1);
//       Wire.endTransmission();	  

//     }
//   }
//   delay(15); 
// }