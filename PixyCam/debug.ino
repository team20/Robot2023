#include <Wire.h>
uint8_t queue[] = {
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42,
    42};
void setup() {
	Wire.begin(0x30);              // join I2C bus with address #8
	Wire.onRequest(requestEvent);  // register event
}

void loop() {
	delay(15);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
	Wire.write(queue, sizeof(que));
}
