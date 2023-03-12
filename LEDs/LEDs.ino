#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x18
#define NUM_BYTES_TO_READ 1
// I2C Master Code for Arduino Nano

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define UPPER_LED_PIN 5
#define LOWER_LED_PIN 6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 33
// Declare our NeoPixel strip object:
Adafruit_NeoPixel upperStrip(LED_COUNT, UPPER_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lowerStrip(LED_COUNT, LOWER_LED_PIN, NEO_GRB + NEO_KHZ800);

// Argument 1 = Number of pixels in NeoPixel upperStrip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed, refer to Adafruit_NeoPixel.h
// for additional flags(in the event the LEDs don't display the color you want, you
// probably need a different bitstream):
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_BRG     Pixels are wired for BRG bitstream (whatever LED upperStrip 2023 uses)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
int colorIndex = 0; // frame variable, changes from loop
int pattern = -1;   // pattern led upperStrips are on, read in from master/robot
long startTime = -1;
const long endTime = 30000;                                 // timer variables for endgame
int DHpin = 8;                                              // idk, I copied this blindly, I don't think this does anything
byte dat[5];                                                // same as above
uint32_t upperStripTeamColor = upperStrip.Color(0, 255, 0); // color of team, default to green, can be set by master/robot to alliance color
uint32_t lowerStripTeamColor = upperStrip.Color(0, 255, 0); // color of team, default to green, can be set by master/robot to alliance color
uint32_t lowerStripRedColor = lowerStrip.Color(255, 0, 0);
uint32_t upperStripTeamColor = upperStrip.Color(255, 0, 0);

uint32_t upperMovingGreenRedGradient(int c, int i)
{ // pixel depends on ratio of red and green
  return (upperStrip.Color(255 * ((i + c) % LED_COUNT) / LED_COUNT, 0, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
uint32_t lowerMovingGreenRedGradient(int c, int i)
{ // pixel depends on ratio of red and green
  return (lowerStrip.Color(255 * ((i + c) % LED_COUNT) / LED_COUNT, 0, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
uint32_t upperMovingGreenBlueGradient(int c, int i)
{ // pixel depends on ratio of green and blue
  return (upperStrip.Color(0, 255 * ((i + c) % LED_COUNT) / LED_COUNT, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
uint32_t lowerMovingGreenBlueGradient(int c, int i)
{ // pixel depends on ratio of green and blue
  return (lowerStrip.Color(0, 255 * ((i + c) % LED_COUNT) / LED_COUNT, 255 * (LED_COUNT - (i + c) % LED_COUNT) / LED_COUNT % 255));
}
/// @brief Makes an LED strip alternate colors
/// @param i The LED number
/// @param color3 The color of an even numbered LED
/// @param color4 The color of an odd numbered LED
/// @return The color of the LED
uint32_t BlinkingLights(int i, uint32_t color3, uint32_t color4)
{
  if (i % 2 == 0)
  {
    return (color3);
  }
  return (color4);
}

uint32_t HSV2RGB(uint32_t h, double s, double v)
{ // converts HSV colors to RGB integer, used for rainbows
  // note: idea is copied from google, math is created from thin air, can be trusted, cannot be explained
  h %= 360;
  double c = v * s;                                                                            // vs proportion
  double x = c * (1 - abs((h / 60) % 2 - 1));                                                  // assigning x
  double m = v - c;                                                                            // remaining c-v
  uint8_t r = (uint8_t)(((abs(180 - h) > 120) * c + (abs(180 - h) > 60) * (x - c) + m) * 255); // red
  uint8_t g = (uint8_t)(((abs(120 - h) < 60) * c + (abs(120 - h) < 120) * (x - c) + m) * 255); // green
  uint8_t b = (uint8_t)(((abs(240 - h) < 60) * c + (abs(240 - h) < 120) * (x - c) + m) * 255); // blue
  return (upperStrip.Color(r, g, b));                                                          // convert to uint32_t
}
uint32_t RainbowColor(int c, int i)
{ // solid rainbow, change with c
  return (HSV2RGB((c * 18) % 360, 1, 1));
  // return(upperStrip.Color(0,255,0));
}
uint32_t SeizureRainbowColor(int c, int i)
{ // Rainbow but blinking
  if (c % 2 == 0)
    return (RainbowColor(c / 2, i));
  return (upperStrip.Color(0, 0, 0));
}
uint32_t MovingRainbow(int c, int i)
{ // Moving rainbow
  return (RainbowColor(c + i, i));
}
uint32_t Timer(int c, int i, uint32_t color)
{ // user timer proportion to light up specific pixel
  if (c % 2 == 0 && 2 > (i + c) % int(LED_COUNT * (1 - (millis() - startTime) / endTime)) && i * endTime > LED_COUNT * (millis() - startTime))
  {
    return (color);
  }
  return (upperStrip.Color(0, 0, 0));
}

// setup() function -- runs once at startup --------------------------------
void setup()
{
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  lowerLEDs.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  lowerLEDs.show();             // Turn OFF all pixels ASAP
  lowerLEDs.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)

  upperLEDs.begin();
  upperLEDs.show();
  upperLEDs.setBrightness(255);

  Wire.begin(0x18); // begin I2C
  Serial.begin(9600);
  // Call receiveEvent when data comes in over I2C
  Wire.onReceive(receiveEvent);
}
void loop()
{
  byte x = Wire.read();
  if (Serial.available())
  {
    // If Input exists
    //  Transmit I2C data request
    Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
    Wire.write(NUM_BYTES_TO_READ);
    Wire.endTransmission();
    // Recieve the echoed value back
    Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);              // Begin transmitting to navX-Sensor
    Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ); // Send number of bytes to read     delay(1);
    Wire.endTransmission();                                                   // Stop transmitting
  }
  switch (pattern)
  {       // sets up lights to patterns
          // note: every function returns a color based on colorIndex, the pixel index, and optional color parameters.
          // the for loops set the pixels to have their corrseponding colors based on the pattern function on the colorIndex frame
  case 8: // reset code
    startTime = -1;
    colorIndex = 0;
    pattern = -1;
  case 9: // blinking yellow
    for (int i = 0; i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, BlinkingLights(colorIndex, i, upperStrip.Color(245, 149, 24), upperStrip.Color(0, 0, 0)));
      lowerStrip.setPixelColor(i, BlinkingLights(colorIndex, i, lowerStrip.Color(245, 149, 24), lowerStrip.Color(0, 0, 0)));
    }
    delay(150);
    break;
  case 10: // blinking purple
    for (int i = 0; i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, BlinkingLights(colorIndex, i, upperStrip.Color(230, 0, 255), upperStrip.Color(0, 0, 0)));
      lowerStrip.setPixelColor(i, BlinkingLights(colorIndex, i, lowerStrip.Color(230, 0, 255), lowerStrip.Color(0, 0, 0)));
    }
    delay(150);
    break;
  case 11: // moving green and red gradient
    for (int i = 0; i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, upperMovingGreenRedGradient(colorIndex, i));
      lowerStrip.setPixelColor(i, lowerMovingGreenRedGradient(colorIndex, i));
    }
    delay(25);
    break;
  case 12: // moving green and blue gradient
    for (int i = 0; i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, upperMovingGreenBlueGradient(colorIndex, i));
      lowerStrip.setPixelColor(i, lowerMovingGreenBlueGradient(colorIndex, i));
    }
    delay(25);
    break;
  case 13:
    for (int i = 0l i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, upperStripRedColor);
      lowerStrip.setPixelColor(i, lowerStripRedColor);
    }
    delay(50);
    break;
  default: // display team/alliance color
    for (int i = 0; i < LED_COUNT; i++)
    {
      upperStrip.setPixelColor(i, upperStripTeamColor);
      lowerStrip.setPixelColor(i, lowerStripTeamColor);
    }
    delay(50);
    break;
  }
  upperStrip.show(); // show
  lowerStrip.show();
  colorIndex++; // next frame
}
void receiveEvent(int howMany)
{
  byte x = Wire.read();
  pattern = x;
  // first code must be non-zero multiple of 10
  if (pattern == 8 && Wire.available())
  {                  // for code 90 reset, check if new alliance color being set
    x = Wire.read(); // read in alliance code
    if (x == 11)
    { // red alliance
      upperStripTeamColor = upperStrip.Color(255, 0, 0);
      lowerStripTeamColor = lowerStrip.Color(255, 0, 0);
    }
    else if (x == 12)
    { // blue alliance
      upperStripTeamColor = upperStrip.Color(0, 0, 255);
      lowerStripTeamColor = lowerStrip.Color(0, 0, 255);
    }
  }
}