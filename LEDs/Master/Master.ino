#include <Wire.h>

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8
// I2C Master code for Arduino Nano

int ledState = 0;

void setup()
{
    Wire.begin();       // start i2c
    Serial.begin(9600); // start serial input
}

void loop()
{ // send codes directly from serial
    if (Serial.available())
    { // if input exists
        /* Transmit I2C data request */
        Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
        Wire.write(NUM_BYTES_TO_READ);                               // Send number of bytes to read
        Wire.endTransmission();                                      // Stop transmitting
        /* Receive the echoed value back */
        Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);              // Begin transmitting to navX-Sensor
        Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ); // Send number of bytes to read
        delay(1);
        Wire.endTransmission(); // Stop transmitting
    }
}
