/*
 *
 * TVR Cerbera Door Control ECU Arduino replacement
 * Alex Judd 31/12/2021
 *
 * /

/*
 * Constants
 */

const int windowDownPin = 13; // the pin numbers for the Window Down relay
const int windowUpPin = 14; // the pin numbers for the Window Up relay
const int doorLatchPin = 15; // the pin numbers for the Door Latch relay
const int doorLockPin = 16; // the pin numbers for the Door Lock relay
const int bootLatchPin = 17; // the pin numbers for the Boot Latch relay
const int bootLockPin = 18; // the pin numbers for the Boot Lock relay
const int interiorLightsPin = 19; // the pin numbers for the Interior Lights relay

const int externalDoorButtonPin =  20; // the pin numbers for the External Door button
const int internalDoorButtonPin =  21; // the pin numbers for the Internal Door button
const int windowDownButtonPin = 22; // the pin numbers for the Window Down button
const int windowUpButtonPin = 23; // the pin numbers for the Window Up button
const int bootOpenButtonPin = 24; // the pin number for the Boot Open button

const int roadSpeedPin = 26; // the pin number for the road speed
const int alarmInputPin = 27; // the pin number for the alarm input

const int windowDropLength = 5000; // number of milliseconds to drop the window for
const int unlockLength = 2000; // number of milliseconds to wait for the door to unlock
const int unlatchLength = 5000; // number of milliseconds to open the door to unlatch

/*
 * Variables
 */

byte alarmState = LOW;
byte lightsState = LOW;

bool alarmed = false;
bool doorUnlocked = false;
bool doorOpen = false;
bool buttonPressed = false;
bool openingDoor = false;

int roadSpeed = 0;
int windowPosition = 0;

/*
 * Setup
 */

void setup() {

	/* relay outputs */
	pinMode(windowDownPin, OUTPUT); // initialize pin 13 [Window Down] as an output
	pinMode(windowUpPin, OUTPUT); // initialize pin 14 [Window Up] as an output
	pinMode(doorLatchPin, OUTPUT); // initialize pin 15 [Door Latch] as an output
	pinMode(doorLockPin, OUTPUT); // initialize pin 16 [Door Lock] as an output
	pinMode(bootLatchPin, OUTPUT); // initialize pin 17 [Boot Latch] as an output
	pinMode(bootLockPin, OUTPUT); // initialize pin 18 [Boot Lock] as an output
	pinMode(interiorLightsPin, OUTPUT);

	/* switch inputs */
	pinMode(externalDoorButtonPin, INPUT); // initialize pin 20 [Outdoor door button] as an input
	pinMode(internalDoorButtonPin, INPUT); // initialize pin 21 [Indoor door button] as an input
	pinMode(windowDownButtonPin, INPUT); // initialize pin 22 [Window Down button] as an input
	pinMode(windowUpButtonPin, INPUT); // initialize pin 23 [Window Up button] as an input
	pinMode(bootOpenButtonPin, INPUT); // initialize pin 24 [Boot Open button] as an input

	/* external data */
	pinMode(roadSpeedPin, INPUT);
	pinMode(alarmInputPin, INPUT);
}

/*
 * Main Loop
 */

void loop() {
	//check alarm state
	updateAlarmState();
	readDoorButtons();
	digitalWrite(13, HIGH);
	delay(4000);
	digitalWrite(13, LOW);
	delay(4000);
}

/*
 * Functions
 */

void updateAlarmState() {
	if(digitalRead(alarmInputPin) == HIGH) {
		alarmed = true;
	}
}

void updateButtonState() {
		if((digitalRead(externalDoorButtonPin) == LOW) || (digitalRead(internalDoorButtonPin) == LOW)) {
			buttonPressed = true;
		} else {
			buttonPressed = false;
		}
}

void readDoorButtons() {
	if (!alarmed) {
		if(buttonPressed && !openingDoor) {
			openingDoor = true;
			turnOnLights();
			dropWindow();
			unlockDoor();
			unlatchDoor();
			openingDoor = false;
		}
	}
}

void turnOnLights() {

}

void dropWindow() {

}

void unlockDoor() {

}

void unlatchDoor() {

}