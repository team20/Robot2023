// To upload to Arduino you must select your board (Arduino Nano) 
// this is for robot 2023 and you also need a MINI-B USB Cable to plug into
// the arduino, then into your computer (pick the port)
// then find the arrow button at the top near where you choose your port and click to upload 

#include <Adafruit_NeoPixel.h> // imports
#include <Wire.h>
#ifdef __AVR__
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32 // the i2c address (arduino has an file in examples to find this address)
#define NUM_BYTES_TO_READ 1 // this can be changed to whatever but 1 byte is enough, we don't need anymore
//I2C Master Code for Arduino Nano

int ledState = 0;

#define LED_PIN 5 // Which pin on the Arduino is connected to the NeoPixels?

#define LED_COUNT 18 // How many NeoPixels are attached to the Arduino? 
// TODO: Might want to recount? Apparently doesn't work on one side? Might want to retest this just in case
// but since it will be different LEDs on actual robot shouldn't worry about it too much

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products) - THIS IS OURS (2023 L-Board)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// setup() function -- runs once at startup --------------------------------

int colorIndex = 0;  //frame variable, chnages from loop
int pattern = -1;    //pattern led strips are on, read in from master/robot
long startTime = -1;
const long endTime = 30000;                   //timer variables for endgame
int DHpin = 8;                                //idk, I copied this blindly, I don't think this does anything
byte dat[5];                                  //same as above
uint32_t teamColor = strip.Color(0, 0, 255);  //color of team, default to green, can be set by master/robot to alliance color

uint32_t MovingRedGreenGradient(int c, int i) {  //pixel depends on ratio of red and green
  return (strip.Color(255 * ((i + c) % LED_COUNT) / LED_COUNT, 0, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
uint32_t MovingGreenBlueGradient(int c, int i) {  //pixel depends on ratio of green and blue
  return (strip.Color(0, 255 * ((i + c) % LED_COUNT) / LED_COUNT, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
uint32_t TheaterLights(int c, int i, uint32_t color1, uint32_t color2) {  //pixel on color 1 or 2 depending on frame
  if (i % 2 == c % 2) { return (color1); }
  return (color2);
}
uint32_t HSV2RGB(uint32_t h, double s, double v) {  //converts HSV colors to RGB integer, used for rainbows
  //note: idea is copied from google, math is created from thin air, can be trusted, cannot be explained
  h %= 360;
  double c = v * s;                                                                             //vs proportion
  double x = c * (1 - abs((h / 60) % 2 - 1));                                                   //assigning x
  double m = v - c;                                                                             //remaining c-v
  uint8_t r = (uint8_t)(((abs(180 - h) > 120) * c + (abs(180 - h) > 60) * (x - c) + m) * 255);  //red
  uint8_t g = (uint8_t)(((abs(120 - h) < 60) * c + (abs(120 - h) < 120) * (x - c) + m) * 255);  //green
  uint8_t b = (uint8_t)(((abs(240 - h) < 60) * c + (abs(240 - h) < 120) * (x - c) + m) * 255);  //blue
  return (strip.Color(r, g, b));                                                                //convert to uint32_t
}
uint32_t RainbowColor(int c, int i) {  //solid rainbow, change with c
  return (HSV2RGB((c * 18) % 360, 1, 1));
  //return(strip.Color(0,255,0));
}
uint32_t SeizureRainbowColor(int c, int i) {  //Rainbow but blinking
  if (c % 2 == 0) return (RainbowColor(c / 2, i));
  return (strip.Color(0, 0, 0));
}
uint32_t MovingRainbow(int c, int i) {  //Moving rainbow
  return (RainbowColor(c + i, i));
}
uint32_t Timer(int c, int i, uint32_t color) {  //user timer proportion to light up specific pixel
  if (c % 2 == 0 && 2 > (i + c) % int(LED_COUNT * (1 - (millis() - startTime) / endTime)) && i * endTime > LED_COUNT * (millis() - startTime)) { return (color); }
  return (strip.Color(0, 0, 0));
}
void setup() {
  Serial.println("Starting");
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  Serial.print("Initialized");
  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
  Wire.begin(0x18);         //begin I2C
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);  // set up data slave recieved
}
void loop() {
  // Serial.println("Running");
  //  pattern=(int)((millis()-testStart)/testInc)-1;if(pattern>7){pattern=7;} // used for timed demo
  byte x = Wire.read();
  if (Serial.available()) {
    //If Input exists
    // Transmit I2C data request
    Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
    Wire.write(NUM_BYTES_TO_READ);
    Wire.endTransmission();
    // Recieve the echoed value back
    Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);               // Begin transmitting to navX-Sensor
    Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);  // Send number of bytes to read     delay(1);
    Wire.endTransmission();                                                    // Stop transmitting
  }
  switch (pattern) {  //sets up lights to patterns
                      //  note: every function returns a color based on colorIndex, the pixel index, and optional color parameters.
                      //         the for loops set the pixels to have their corrseponding colors based on the pattern function on the colorIndex frame
    case 8:           //reset code
      startTime = -1;
      colorIndex = 0;
      pattern = -1;
    case 17:  //green back and forth timer
      if (startTime < 0) { startTime = millis(); }
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, Timer(colorIndex, i, strip.Color(0, 0, 250))); }
      delay(10);
      break;
    case 16:  //blue back and forth timer
      if (startTime < 0) { startTime = millis(); }
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, Timer(colorIndex, i, strip.Color(0, 250, 0))); }
      delay(10);
      break;
    case 15:  //moving red and green gradient
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, MovingRedGreenGradient(colorIndex, i)); }
      delay(25);
      break;
    case 14:  //green theater lights
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, TheaterLights(colorIndex, i, strip.Color(0, 0, 250), strip.Color(0, 0, 0))); }
      delay(100);
      break;
    case 13:  //moving green and blue gradient
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, MovingGreenBlueGradient(colorIndex, i)); }
      delay(25);
      break;
    case 12:  //red theater lights
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, TheaterLights(colorIndex, i, strip.Color(250, 0, 0), strip.Color(0, 0, 0))); }
      delay(100);
      break;
    case 11:  //blue theater lights
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, TheaterLights(colorIndex, i, strip.Color(0, 250, 0), strip.Color(0, 0, 0))); }
      delay(100);
      break;
    case 10:  //orange theater lights
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, TheaterLights(colorIndex, i, strip.Color(255, 0, 115), strip.Color(0, 0, 0))); }
      delay(100);
      break;
    default:  //display team/alliance color
      for (int i = 0; i < LED_COUNT; i++) { strip.setPixelColor(i, teamColor); }
      delay(50);
      break;
  }
  strip.show();  //show
  colorIndex++;  //next frame
}
void receiveEvent(int howMany) {
  byte x = Wire.read();
  // Serial.println(x);
  // Serial.print(0);
  pattern = x;
  //first code must be non-zero multiple of 10
  if (pattern == 8 && Wire.available()) {  //for code 90 reset, check if new alliance color being set
    x = Wire.read();                       //read in alliance code
    if (x == 5) {                          //red alliance
      teamColor = strip.Color(255, 0, 0);
    } else if (x == 15) {  //blue alliance
      teamColor = strip.Color(0, 255, 0);
    }
  }
}