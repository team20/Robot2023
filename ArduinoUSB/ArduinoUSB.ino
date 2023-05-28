byte data[];
void setup() {
	Serial.begin(250000);
}
void loop() {
	Serial.write("Arduino");
}
void serialEvent() {
	Serial.readBytes(data, 7);
}