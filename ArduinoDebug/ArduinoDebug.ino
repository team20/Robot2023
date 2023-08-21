
#include <Wire.h>
// clang-format off
uint8_t queue[] = {
	42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53,
	54, 55, 56, 57, 58, 59,
	60, 61, 62, 63, 64, 65,
	66, 67, 68, 69, 70, 71
};
// clang-format on
void setup() {
	Wire.begin(0x18);              // join I2C bus with address #8
	Wire.onRequest(requestEvent);  // register event
	Serial.begin(9600);
}

void loop() {
	delay(15);
	Serial.println("Hi!");
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
	Serial.println("Request received!");
	Wire.write(queue, sizeof(queue));
}