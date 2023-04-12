#include <Pixy2.h>
#include <Wire.h>

Pixy2 pixy;

void setup() {
	Serial.begin(9600);
	pixy.init();
	Wire.begin(0x30);
	Wire.onRequest(sendData);
}
void sendData() {
	// Send a message to client:
	//    9 byte block:
	//       signature: 1 byte
	//       x: 2 bytes
	//       y: 1 byte
	//       width: 2 bytes
	//       height: 1 byte
	//       index: 1 byte
	//       age: 1 byte

	Serial.println("STARTING");
	Wire.beginTransmission(0x32);
	// Wire.write(static_cast<uint8_t>(pixy.ccc.numBlocks));
	int nSent = 0;
	if (pixy.ccc.numBlocks) {
		for (int i = 0; (i < pixy.ccc.numBlocks) && (i < 4); ++i) {
			Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_signature));

			uint16_t x = pixy.ccc.blocks[i].m_x;
			Wire.write((uint8_t *)&x, sizeof(x));

			Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_y));

			uint16_t width = pixy.ccc.blocks[i].m_width;
			Wire.write((uint8_t *)&width, sizeof(width));

			Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_height));

			Wire.write(pixy.ccc.blocks[i].m_index);
			Wire.write(pixy.ccc.blocks[i].m_age);
			++nSent;
		}
	}
	if (nSent < 4) {
		uint8_t *send = new uint8_t[12 * (4 - nSent)];
		Wire.write(send, 12 * (4 - nSent));
	}
	Wire.endTransmission();
	Serial.println("ENDED");
}

void loop() {
	pixy.ccc.getBlocks();
	if (pixy.ccc.numBlocks) {
		uint16_t sig = hton(pixy.ccc.blocks[0].m_signature);
		Serial.println(sig);
	}
	delay(15);
}

uint16_t hton(uint16_t hostint) {
	return ((hostint << 8) & 0xFF00) | ((hostint >> 8) & 0x00FF);
}