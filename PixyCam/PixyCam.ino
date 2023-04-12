#include <Wire.h>
#include <Pixy2.h>

Pixy2 pixy;

void setup() {
  Serial.begin(9600);
	 pixy.init();
  Wire.begin(0x30);
  Wire.onRequest(sendData);
}
void sendData(){

  // Send a message to client:

  //  [ 
  //    numBlocks: 2 bytes 
  //    12 byte block:
  //       signature: 2 bytes
  //       x: 2 bytes
  //       y: 2 bytes
  //       width: 2 bytes
  //       height: 2 bytes
  //       index: 1 byte
  //       age: 1 byte

  Serial.println("STARTING");
  Wire.beginTransmission(0x32);
  //Wire.write(static_cast<uint8_t>(pixy.ccc.numBlocks)); 
  int nSent = 0;
  if (pixy.ccc.numBlocks) {
    for (int i=0; (i<pixy.ccc.numBlocks) && (i<4); ++i){
      
		 uint16_t sig = hton(pixy.ccc.blocks[i].m_signature);
      Wire.write((uint8_t *) &sig, sizeof(sig));
      
		 uint16_t x = hton(pixy.ccc.blocks[i].m_x);
      Wire.write((uint8_t *) &x, sizeof(x));

		 uint16_t y = hton(pixy.ccc.blocks[i].m_y);
      Wire.write((uint8_t *) &y, sizeof(y));

		 uint16_t width = hton(pixy.ccc.blocks[i].m_width);
      Wire.write((uint8_t *) &width, sizeof(width));

		 uint16_t height = hton(pixy.ccc.blocks[i].m_height);
      Wire.write((uint8_t *) &height, sizeof(height));
      
      Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_index));
      Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_age));
      ++nSent;      
    }
    
  }
  if(nSent < 4){
      uint8_t *send = new uint8_t[12*(4-nSent)];
      Wire.write(send, 12*(4-nSent));      
    }
  Wire.endTransmission();
    Serial.println("ENDED");	  
}

void loop() {
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks){
    uint16_t sig = hton(pixy.ccc.blocks[0].m_signature);
    Serial.println(sig);
  }
  delay(15); 
}

uint16_t hton(uint16_t hostint){
  return ((hostint << 8) & 0xFF00) | ((hostint >> 8) & 0x00FF);
}