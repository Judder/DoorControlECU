void setup() {
	pinMode(12, OUTPUT); // initialize pin 12 as an output
	pinMode(13, OUTPUT); // initialize pin 13 as an output
}

void loop() {
	digitalWrite(13, HIGH);
	delay(4000);
	digitalWrite(13, LOW);
	delay(4000);
}