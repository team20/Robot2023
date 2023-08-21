byte data[];
void setup() {
	Serial.begin(9600);
}
void loop() {
	Serial.write("Arduino");
	delay(40);
}
void serialEvent() {
	Serial.readBytes(data, 7);
}