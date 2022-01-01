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

const int driverWindowDownPin = 2; // the pin numbers for the Window Down relay
const int driverWindowUpPin = 3; // the pin numbers for the Window Up relay
const int driverDoorLatchPin = 4; // the pin numbers for the Door Latch relay
const int driverDoorLockPin = 5; // the pin numbers for the Door Lock relay

const int passengerWindowDownPin = 6; // the pin numbers for the Window Down relay
const int passengerWindowUpPin = 7; // the pin numbers for the Window Up relay
const int passengerDoorLatchPin = 8; // the pin numbers for the Door Latch relay
const int passengerDoorLockPin = 9; // the pin numbers for the Door Lock relay

const int bootLatchPin = 10; // the pin numbers for the Boot Latch relay
const int bootLockPin = 11; // the pin numbers for the Boot Lock relay

const int interiorLightsPin = 12; // the pin numbers for the Interior Lights relay

/* Digital inputs */

const int driverExternalDoorButtonPin =  13; // the pin numbers for the External Door button
const int driverInternalDoorButtonPin =  14; // the pin numbers for the Internal Door button
const int driverWindowDownButtonPin = 15; // the pin numbers for the Window Down button
const int driverWindowUpButtonPin = 16; // the pin numbers for the Window Up button
const int driverPinSwitch = 17; // the pin number for the door open / closed switch

const int passengerExternalDoorButtonPin =  18; // the pin numbers for the External Door button
const int passengerInternalDoorButtonPin =  19; // the pin numbers for the Internal Door button
const int passengerWindowDownButtonPin = 20; // the pin numbers for the Window Down button
const int passengerWindowUpButtonPin = 21; // the pin numbers for the Window Up button
const int passengerPinSwitch = 22; // the pin number for the door open / closed switch

const int bootOpenButtonPin = 23; // the pin number for the Boot Open button

const int alarmInputPin = 24; // the pin number for the alarm input

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
	pinMode(driverWindowDownPin, OUTPUT); // initialize pin 13 [Window Down] as an output
	pinMode(driverWindowUpPin, OUTPUT); // initialize pin 14 [Window Up] as an output
	pinMode(driverDoorLatchPin, OUTPUT); // initialize pin 15 [Door Latch] as an output
	pinMode(driverDoorLockPin, OUTPUT); // initialize pin 16 [Door Lock] as an output
	pinMode(passengerWindowDownPin, OUTPUT); // initialize pin 13 [Window Down] as an output
  pinMode(passengerWindowUpPin, OUTPUT); // initialize pin 14 [Window Up] as an output
  pinMode(passengerDoorLatchPin, OUTPUT); // initialize pin 15 [Door Latch] as an output
  pinMode(passengerDoorLockPin, OUTPUT); // initialize pin 16 [Door Lock] as an output
  pinMode(bootLatchPin, OUTPUT); // initialize pin 17 [Boot Latch] as an output
	pinMode(bootLockPin, OUTPUT); // initialize pin 18 [Boot Lock] as an output
	pinMode(interiorLightsPin, OUTPUT);

	/* switch inputs */
	pinMode(driverExternalDoorButtonPin, INPUT_PULLUP); // initialize pin 20 [Outdoor door button] as an input
	pinMode(driverInternalDoorButtonPin, INPUT_PULLUP); // initialize pin 21 [Indoor door button] as an input
	pinMode(driverWindowDownButtonPin, INPUT_PULLUP); // initialize pin 22 [Window Down button] as an input
	pinMode(driverWindowUpButtonPin, INPUT_PULLUP); // initialize pin 23 [Window Up button] as an input
  pinMode(passengerExternalDoorButtonPin, INPUT_PULLUP); // initialize pin 20 [Outdoor door button] as an input
  pinMode(passengerInternalDoorButtonPin, INPUT_PULLUP); // initialize pin 21 [Indoor door button] as an input
  pinMode(passengerWindowDownButtonPin, INPUT_PULLUP); // initialize pin 22 [Window Down button] as an input
  pinMode(passengerWindowUpButtonPin, INPUT_PULLUP); // initialize pin 23 [Window Up button] as an input
	pinMode(bootOpenButtonPin, INPUT_PULLUP); // initialize pin 24 [Boot Open button] as an input

	/* external data */
	pinMode(roadSpeedPin, INPUT);
	pinMode(alarmInputPin, INPUT);
}

/*
 * Main Loop
 */

void loop() {

	#ifdef testMode
		#ifdef lightTestMode
      lightsTest();
		#endif
		#ifdef windowTestMode
      windowButtonsTest();
		#endif
		#ifdef doorTestMode
      doorButtonsTest();
		#endif
		#ifdef bootTestMode
      bootButtonTest();
		#endif
		#ifdef speedTestMode
			speedTest();
		#endif
		#ifdef alarmTestMode
			alarmTest();
		#endif
	#else
		//check alarm state
		updateAlarmState();
		readDriverDoorButtons();
		readDriverWindowButtons();
		readPassengerDoorButtons();
		readPassengerWindowButtons();
		readBootButton();

	#endif
}

/*
 * Functions
 */

bool debounceButton(int reading,
					int buttonState,
					int lastButtonState = LOW,
					unsigned long lastDebounceTime = 0,
					unsigned long debounceDelay = 50) {

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (reading != lastButtonState) {
		// reset the debouncing timer
		lastDebounceTime = millis();
	}

	if ((millis() - lastDebounceTime) > debounceDelay) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:

		// if the button state has changed:
		if (reading != buttonState) {
			return true;
		} else {
			return false;
		}
	}
}

void updateAlarmState() {
	if(digitalRead(alarmInputPin) == HIGH) {
		alarmed = true;
	} else {
		alarmed = false;
	}
}

void readDriverDoorButtons() {
	if (!alarmed) {
		int reading;

		bool driverExternalButtonChanged = false;
		reading = digitalRead(driverExternalDoorButtonPin);
		driverExternalButtonChanged = debounceButton(reading,
		        driverExternalDoorButtonState,
		        driverExternalDoorLastButtonState,
		        lastDriverExternalDoorButtonDebounceTime);
		if (driverExternalButtonChanged) {
			driverExternalDoorButtonState = !(driverExternalDoorButtonState);
		}

		bool driverInternalButtonChanged = false;
		reading = digitalRead(driverInternalDoorButtonPin);
		driverInternalButtonChanged = debounceButton(reading,
						driverInternalDoorButtonState,
						driverInternalDoorLastButtonState,
						lastDriverInternalDoorButtonDebounceTime);
		if (driverInternalButtonChanged) {
			driverInternalDoorButtonState = !(driverInternalDoorButtonState);
		}

		if (driverExternalButtonChanged || driverInternalButtonChanged) {
			driverOpeningDoor = true;
			turnOnLights();
			driverDropWindow();
			driverUnlockDoor();
			driverUnlatchDoor();
			driverOpeningDoor = false;
		}
	}
}

void readDriverWindowButtons() {
}

void readPassengerDoorButtons() {
	if (!alarmed) {
		int reading;

		bool passengerExternalButtonChanged = false;
		reading = digitalRead(passengerExternalDoorButtonPin);
		passengerExternalButtonChanged = debounceButton(reading,
						passengerExternalDoorButtonState,
						passengerExternalDoorLastButtonState,
						lastPassengerExternalDoorButtonDebounceTime);
		if (passengerExternalButtonChanged) {
			passengerExternalDoorButtonState = !(passengerExternalDoorButtonState);
		}

		bool passengerInternalButtonChanged = false;
		reading = digitalRead(passengerInternalDoorButtonPin);
		passengerInternalButtonChanged = debounceButton(reading,
						passengerInternalDoorButtonState,
						passengerInternalDoorLastButtonState,
						lastPassengerInternalDoorButtonDebounceTime);
		if (passengerInternalButtonChanged) {
			passengerInternalDoorButtonState = !(passengerInternalDoorButtonState);
		}

		if (passengerExternalButtonChanged || passengerInternalButtonChanged) {
			passengerOpeningDoor = true;
			turnOnLights();
			passengerDropWindow();
			passengerUnlockDoor();
			passengerUnlatchDoor();
			passengerOpeningDoor = false;
		}
	}
}

void readPassengerWindowButtons() {
}

void turnOnLights() {
		digitalWrite(interiorLightsPin, HIGH);
}

void turnOffLights() {
		digitalWrite(interiorLightsPin, LOW);
}

void driverDropWindow() {

}

void driverUnlockDoor() {

}

void driverUnlatchDoor() {

}

void passengerDropWindow() {

}

void passengerUnlockDoor() {

}

void passengerUnlatchDoor() {

}

void doorButtonsTest() {
    int reading;
    bool driverExternalButtonChanged = false;
    reading = digitalRead(driverExternalDoorButtonPin);
    driverExternalButtonChanged = debounceButton(reading,
            driverExternalDoorButtonState,
            driverExternalDoorLastButtonState,
            lastDriverExternalDoorButtonDebounceTime);
    if (driverExternalButtonChanged) {
      driverExternalDoorButtonState = !(driverExternalDoorButtonState);
      Serial.println("Drivers external door button: " + driverExternalDoorButtonState);
    }

    bool driverInternalButtonChanged = false;
    reading = digitalRead(driverInternalDoorButtonPin);
    driverInternalButtonChanged = debounceButton(reading,
            driverInternalDoorButtonState,
            driverInternalDoorLastButtonState,
            lastDriverInternalDoorButtonDebounceTime);
    if (driverInternalButtonChanged) {
      driverInternalDoorButtonState = !(driverInternalDoorButtonState);
      Serial.println("Driver internal door button: " + driverInternalDoorButtonState);
    }
}

void windowButtonsTest() {
}

void lightsTest() {
      digitalWrite(interiorLightsPin, HIGH);
      Serial.println("Interior Light On");
      delay(4000);
      digitalWrite(interiorLightsPin, LOW);
      Serial.println("Interior Light Off");
      delay(4000);
}

void speedTest() {
      int roadSpeed = analogRead(roadSpeedPin);
      Serial.println("Road speed: " + roadSpeed);
      delay(1000);
}

void alarmTest() {
      int alarm = digitalRead(alarmInputPin);
      Serial.println("Alarm state: " + alarmInputPin);
      delay(1000);
}

void bootButtonTest() {
  
}
