/*
 *
 * TVR Cerbera Door Control ECU Arduino replacement
 * (c) Alex Judd 31/12/2021
 * alex@alexjudd.com
 * /

/*
 * Defines
 */

#define testMode
#define lightTestMode
//#define windowTestMode
//#define doorTestMode
//#define bootTestMode
//#define speedTestMode
//#define alarmTestMode

/*
 * Constants
 */

/* Relay outputs */

const int driverWindowDownPin = 9; // the pin numbers for the Window Down relay
const int driverWindowUpPin = 10; // the pin numbers for the Window Up relay
const int driverDoorLatchPin = 11; // the pin numbers for the Door Latch relay
const int driverDoorLockPin = 12; // the pin numbers for the Door Lock relay

const int passengerWindowDownPin = 13; // the pin numbers for the Window Down relay
const int passengerWindowUpPin = 14; // the pin numbers for the Window Up relay
const int passengerDoorLatchPin = 15; // the pin numbers for the Door Latch relay
const int passengerDoorLockPin = 16; // the pin numbers for the Door Lock relay

const int bootLatchPin = 17; // the pin numbers for the Boot Latch relay
const int bootLockPin = 18; // the pin numbers for the Boot Lock relay

const int interiorLightsPin = 19; // the pin numbers for the Interior Lights relay

/* Digital inputs */

const int driverExternalDoorButtonPin =  20; // the pin numbers for the External Door button
const int driverInternalDoorButtonPin =  21; // the pin numbers for the Internal Door button
const int driverWindowDownButtonPin = 22; // the pin numbers for the Window Down button
const int driverWindowUpButtonPin = 23; // the pin numbers for the Window Up button
const int driverPinSwitch = 24; // the pin number for the door open / closed switch

const int passengerExternalDoorButtonPin =  25; // the pin numbers for the External Door button
const int passengerInternalDoorButtonPin =  26; // the pin numbers for the Internal Door button
const int passengerWindowDownButtonPin = 27; // the pin numbers for the Window Down button
const int passengerWindowUpButtonPin = 28; // the pin numbers for the Window Up button
const int passengerPinSwitch = 29; // the pin number for the door open / closed switch

const int bootOpenButtonPin = 30; // the pin number for the Boot Open button

const int alarmInputPin = 31; // the pin number for the alarm input

/* Analogue inputs */

const int roadSpeedPin = A0; // the pin number for the road speed
const int driverWindowPosition = A1;
const int passengerWindowPosition = A2;

const int windowDropLength = 5000; // number of milliseconds to drop the window for
const int unlockLength = 2000; // number of milliseconds to wait for the door to unlock
const int unlatchLength = 5000; // number of milliseconds to open the door to unlatch

/*
 * Variables
 */

byte alarmState = LOW;
byte lightsState = LOW;

bool alarmed = false;

bool driverDoorUnlocked = false;
bool driverDoorOpen = false;
bool driverButtonPressed = false;
bool driverOpeningDoor = false;

bool passengerDoorUnlocked = false;
bool passengerDoorOpen = false;
bool passengerButtonPressed = false;
bool passengerOpeningDoor = false;

int roadSpeed = 0;
int windowPosition = 0;

/*
 * Debounce variables
 *
 *  Each time the input pin goes from LOW to HIGH (e.g. because of a push-button
 * press), the output pin is toggled from LOW to HIGH or HIGH to LOW. There's a
 * minimum delay between toggles to debounce the circuit (i.e. to ignore noise).
 *
 * Reference: https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce 
 */

int driverExternalDoorButtonState;
int driverExternalDoorLastButtonState = HIGH;
int driverInternalDoorButtonState;
int driverInternalDoorLastButtonState = HIGH;
int driverWindowDownButtonState;
int driverWindowDownLastButtonState = HIGH;
int driverWindowUpButtonState;
int driverWindowUpLastButtonState = HIGH;

int passengerExternalDoorButtonState;
int passengerExternalDoorLastButtonState = HIGH;
int passengerInternalDoorButtonState;
int passengerInternalDoorLastButtonState = HIGH;
int passengerWindowDownButtonState;
int passengerWindowDownLastButtonState = HIGH;
int passengerWindowUpButtonState;
int passengerWindowUpLastButtonState = HIGH;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDriverExternalDoorButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastDriverInternalDoorButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastDriverWindowDownButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastDriverWindowUpButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastPassengerExternalDoorButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastPassengerInternalDoorButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastPassengerWindowDownButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastPassengerWindowUpButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

/*
 * Setup
 */

void setup() {
	/* start serial connection */
	Serial.begin(9600);

	/* relay outputs */
	pinMode(windowDownPin, OUTPUT); // initialize pin 13 [Window Down] as an output
	pinMode(windowUpPin, OUTPUT); // initialize pin 14 [Window Up] as an output
	pinMode(doorLatchPin, OUTPUT); // initialize pin 15 [Door Latch] as an output
	pinMode(doorLockPin, OUTPUT); // initialize pin 16 [Door Lock] as an output
	pinMode(bootLatchPin, OUTPUT); // initialize pin 17 [Boot Latch] as an output
	pinMode(bootLockPin, OUTPUT); // initialize pin 18 [Boot Lock] as an output
	pinMode(interiorLightsPin, OUTPUT);

	/* switch inputs */
	pinMode(externalDoorButtonPin, INPUT_PULLUP); // initialize pin 20 [Outdoor door button] as an input
	pinMode(internalDoorButtonPin, INPUT_PULLUP); // initialize pin 21 [Indoor door button] as an input
	pinMode(windowDownButtonPin, INPUT_PULLUP); // initialize pin 22 [Window Down button] as an input
	pinMode(windowUpButtonPin, INPUT_PULLUP); // initialize pin 23 [Window Up button] as an input
	pinMode(bootOpenButtonPin, INPUT_PULLUP); // initialize pin 24 [Boot Open button] as an input

	/* external data */
	pinMode(roadSpeedPin, INPUT);
	pinMode(alarmInputPin, INPUT);
}

/*
 * Main Loop
 */

void loop() {

	#if testMode
		#if lightTestMode
		#endif
		#if windowTestMode
		#endif
		#if doorTestMode
		#endif
		#if bootTestMode
		#endif
		#if speedTestMode
			int roadSpeed = analogRead(roadSpeedPin);
			Serial.println("Road speed: " + roadSpeed);
			delay(1000);
		#endif
		#if alarmTestMode
			int alarm = digitalRead(alarmInputPin);
			Serial.println("Alarm state: " + alarmInputPin);
			delay(1000);
		#endif
	#else
		//check alarm state
		updateAlarmState();
		readDoorButtons();
		readWindowButtons();
		readBootButton();

	#endif
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
		if((digitalRead(driverExternalDoorButtonPin) == LOW) || (digitalRead(driverInternalDoorButtonPin) == LOW)) {
			driverButtonPressed = true;
		} else {
			driverButtonPressed = false;
		}
		if((digitalRead(passengerExternalDoorButtonPin) == LOW) || (digitalRead(passengerInternalDoorButtonPin) == LOW)) {
			passengerButtonPressed = true;
		} else {
			passengerButtonPressed = false;
		}
}

void readDoorButtons() {
	if (!alarmed) {
		if(driverButtonPressed && !driverOpeningDoor) {
			driverOpeningDoor = true;
			turnOnLights();
			driverDropWindow();
			driverUnlockDoor();
			driverUnlatchDoor();
			driverOpeningDoor = false;
		}
	}
}

void turnOnLights() {
		digitalWrite(interiorLightsPin, HIGH);
}

void turnOffLights() {
		digitalWrite(interiorLightsPin, LOW);
}

void dropWindow() {

}

void unlockDoor() {

}

void unlatchDoor() {

}