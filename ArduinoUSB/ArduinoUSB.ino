byte data[];
void setup() {
	Serial.begin(9600);
}
void loop() {
	Serial.write("Arduino");
}
void serialEvent() {
	Serial.readBytes(data, 7);
}