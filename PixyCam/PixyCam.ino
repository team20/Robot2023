#include <Wire.h>
#include <Pixy2.h>

Pixy2 pixy;

void setup() {
  Serial.begin(9600);
  pixy.init();
  Wire.begin(0x30);
  Wire.onRequest(sendData);
}
byte *sent = new byte[48];
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

  //Wire.write(static_cast<uint8_t>(pixy.ccc.numBlocks)); 
  
  Wire.write(sent, sizeof(byte)*48);
}

void loop() {
  int nSent = 0;
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    for (int i=0; (i<pixy.ccc.numBlocks) && (i<4); ++i){


		 uint16_t sig = hton(pixy.ccc.blocks[i].m_signature);



//      Wire.write((byte *) &sig, sizeof(sig));
//      Serial.println(sig);
		 uint16_t x = hton(pixy.ccc.blocks[i].m_x);
//      Wire.write((byte *) &x, sizeof(x));

		 uint16_t y = hton(pixy.ccc.blocks[i].m_y);
//      Wire.write((byte *) &y, sizeof(y));

		 uint16_t width = hton(pixy.ccc.blocks[i].m_width);
//      Wire.write((byte *) &width, sizeof(width));

		 uint16_t height = hton(pixy.ccc.blocks[i].m_height);
      // Wire.write((byte *) &height, sizeof(height));
      
      // Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_index));
      // Wire.write(static_cast<uint8_t>(pixy.ccc.blocks[i].m_age));

      sent[0 + i*12] = ((byte *) &sig)[0];
      sent[1 + i*12] = ((byte *) &sig)[1];
      sent[2 + i*12] = ((byte *) &x)[0];
      sent[3 + i*12] = ((byte *) &x)[1];
      sent[4 + i*12] = ((byte *) &y)[0];
      sent[5 + i*12] = ((byte *) &y)[1];
      sent[6 + i*12] = ((byte *) &width)[0];
      sent[7 + i*12] = ((byte *) &width)[1];
      sent[8 + i*12] = ((byte *) &height)[0];
      sent[9 + i*12] = ((byte *) &height)[1];
      sent[10 + i*12] = static_cast<uint8_t>(pixy.ccc.blocks[i].m_index);
      sent[11 + i*12] = static_cast<uint8_t>(pixy.ccc.blocks[i].m_age);
      ++nSent;      
    }
    for(int i = nSent; i < 4; ++i){
      sent[0 + i*12] = (byte)0;
      sent[1 + i*12] =  byte(0);
      sent[2 + i*12] = byte(0);
      sent[3 + i*12] = byte(0);
      sent[4 + i*12] = byte(0);
      sent[5 + i*12] = byte(0);
      sent[6 + i*12] = byte(0);
      sent[7 + i*12] = byte(0);
      sent[8 + i*12] = byte(0);
      sent[9 + i*12] = byte(0);
      sent[10 + i*12] = byte(0);
      sent[11 + i*12] = byte(0);
      Serial.println(sent[3]);
      ++nSent;  
    }
  }
  delay(15); 
}

uint16_t hton(uint16_t hostint){
  return ((hostint << 8) & 0xFF00) | ((hostint >> 8) & 0x00FF);
}