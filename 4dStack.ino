/*
	Name:       4dStack.ino
	Created:	12-Mar-19 11:24:14 AM
	Author:     Tech Support
*/



#include <U8glib.h>
#include <SyncDriver.h>
#include <MultiDriver.h>
#include <DRV8880.h>
#include <DRV8834.h>
#include <DRV8825.h>
#include <BasicStepperDriver.h>
#include <A4988.h>
#include <SoftI2CMaster.h>
#include <SI2CIO.h>
#include <LiquidCrystal_SR3W.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR1W.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_SI2C.h>
#include <LiquidCrystal_I2C_ByVac.h>
#include <LiquidCrystal.h>
#include <I2CIO.h>
#include <FastIO.h>
#include <Wire.h>
#include <string.h>

// Timer Interrupt
#include <TimerThree.h>

//	LCD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

//	PID 
#include <PID_v1.h>


// LCD
#define I2C_ADDR 0x27
#define Rs_pin 0
#define Rw_pin 1
#define En_pin 2
#define BACKLIGHT_PIN 3
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

// Motor Pins
const int enablePinA = 7;
const int enablePinB = 12;
const int stepPinA = 6;
const int stepPinB = 11;
const int dirPinA = 10;
const int dirPinB = 8;
double speedA = 0;
double speedB = 0;
volatile int activeMotor = stepPinB;

// Syringe Motor Pins
const int enablePinB = 1;

// Non-Blocking Timer
unsigned long t = 0;
//Interrupt Variables
volatile bool motorState = true;
volatile int doneA = 0;
volatile int doneB = 0;
volatile int stepsToTakeA = 0;
volatile int stepsToTakeB = 0;


// Thermistor
#define SERIESRESISTOR 100000
#define THERMISTORPINA A14
#define THERMISTORPINB A15
#define THERMISTORNOMINAL 100000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 7
#define BCOEFFICIENT 3950
int samples[NUMSAMPLES];

// Heat Cartridge
#define heatCartridgeA 4
#define heatCartridgeB 9

// Fans
#define fanA 25
#define fanB 23

//	PID
//double Kp = 30, Ki = 0.2, Kd = 5;
double Kp = 13, Ki = 0.2, Kd = 2;
//double Kp = 90, Ki = 0, Kd = 0;
double SetpointA, InputA, OutputA;
PID myPIDA(&InputA, &OutputA, &SetpointA, Kp, Ki, Kd, DIRECT);

double SetpointB, InputB, OutputB;
PID myPIDB(&InputB, &OutputB, &SetpointB, Kp, Ki, Kd, DIRECT);



void setup()
{
	//	LCD
	lcd.begin(20, 4);
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.home();

	//	Extruder Motors
	pinMode(enablePinA, OUTPUT);
	pinMode(enablePinB, OUTPUT);
	digitalWrite(enablePinA, LOW);
	digitalWrite(enablePinB, LOW);

	pinMode(stepPinA, OUTPUT);
	pinMode(stepPinB, OUTPUT);
	pinMode(dirPinA, OUTPUT);
	pinMode(dirPinB, OUTPUT);
	digitalWrite(dirPinA, LOW);
	digitalWrite(dirPinB, HIGH);

	// Syringe Motors
	pinMode(enablePinC, OUTPUT);
	digitalWrite(enablePinC, LOW);


	//	Interrupts
	Timer3.initialize(500);
	Timer3.attachInterrupt(stepExtruder);

	//	Heat Cartridge
	pinMode(heatCartridgeA, OUTPUT);
	pinMode(heatCartridgeB, OUTPUT);

	// Fans
	pinMode(fanA, OUTPUT);
	pinMode(fanB, OUTPUT);
	digitalWrite(fanA, HIGH);
	digitalWrite(fanB, HIGH);


	//	PID
	InputA = readThermistor(THERMISTORPINA);
	SetpointA = 0;
	myPIDA.SetMode(AUTOMATIC);

	InputB = readThermistor(THERMISTORPINB);
	SetpointB = 0;
	myPIDB.SetMode(AUTOMATIC);

	//	Serial COM
	Serial.begin(9600);
}
int iterations = 0;

void loop()
{
	iterations++;

// COM
	if (Serial.available() > 4)
	{
		const int messageSize = sizeof(char) * 42;
		char incomingByte[messageSize] = "";
		int lastByte = 0;
		char separator[] = ",";
		char *e, *varA, *varB, *varC;
		bool isValid = true;
		char serialInByte;

		do {
			serialInByte = Serial.read();
			incomingByte[lastByte] = serialInByte;
			lastByte++;
		} while (serialInByte != '\n' && Serial.available());
		if (serialInByte != '\n') {
			isValid = false;
		}
		Serial.println(incomingByte);
		if (isValid) {
			e = strtok(incomingByte, separator);
			varA = strtok(NULL, separator);
			varB = strtok(NULL, separator);
			varC = strtok(NULL, separator);
			
			switch (e[0]) {
				case 'a':
					SetpointA = atof(varA);
					speedA = atof(varB);
					stepsToTakeA = atof(varC);
					if (speedA > 0 && activeMotor == stepPinA) {
						noInterrupts();
						Timer3.setPeriod((1/speedA)*1000000);
						interrupts();
					}
					break;
				case 'b':
					SetpointB = atof(varA);
					speedB = atof(varB);
					stepsToTakeB = atof(varC);
					if (speedB > 0 && activeMotor == stepPinB) {
						noInterrupts();
						Timer3.setPeriod((1/speedB) * 1000000);
						interrupts();
					}
					break;
				case 'd':
					if (atof(varA) == 1) {
						noInterrupts();
						activeMotor = stepPinA;
						if (speedA > 0) {
							Timer3.setPeriod((1 / speedA) * 1000000);
						}
						interrupts();
					}
					else {
						noInterrupts();
						activeMotor = stepPinB;
						if (speedB > 0) {
							Timer3.setPeriod((1 / speedB) * 1000000);
						}
						interrupts();
					}
					break;
				case 'c':
					Serial.println("e command");
					int paramA = atof(varA);
					int paramB = atof(varB);
					int fanOrRev = atof(varC);
					if (fanOrRev == 1) {
						if (paramA != 2) {
							digitalWrite(dirPinA, paramA);
						}
						if (paramB != 2) {
							digitalWrite(dirPinB, paramB);
						}
					}
					else {
						if (paramA != 2) {
							digitalWrite(fanA, paramA);
						}
						if (paramB != 2) {
							digitalWrite(fanB, paramB);
						}
					}
					break;
				//case 'c':
				//	Serial.println("c command");
				//	int tempA = atof(varA);
				//	int tempB = atof(varB);
				//	if (tempA != 2) {
				//		digitalWrite(fanA, tempA);
				//	}
				//	if (tempB != 2) {
				//		digitalWrite(fanB, tempB);
				//	}
				//	break;
			}
		}
	}
	
// Temp
	InputA = readThermistor(THERMISTORPINA);
	myPIDA.Compute();
	analogWrite(heatCartridgeA, OutputA);
	InputB = readThermistor(THERMISTORPINB);
	myPIDB.Compute();
	analogWrite(heatCartridgeB, OutputB);

	
// LCD
	
	lcd.setCursor(0, 0);
	lcd.print("Ta: ");
	lcd.print(InputA);
	lcd.print(" / ");
	lcd.print(SetpointA);
	lcd.setCursor(0, 1);
	lcd.print("Sa: ");
	lcd.print(speedA);
	lcd.print(" Hz  ");
	//lcd.print(" / ");
	//lcd.print(doneA);

	lcd.setCursor(0, 2);
	lcd.print("Tb: ");
	lcd.print(InputB);
	lcd.print(" / ");
	lcd.print(SetpointB);
	lcd.setCursor(0, 3);
	lcd.print("Sb: ");
	lcd.print(speedB);
	lcd.print(" Hz  ");
	//lcd.print(" / ");
	//lcd.print(doneB);

// COM
	Serial.print("t");
	Serial.print(InputA, 2);
	Serial.print("t");
	Serial.println(InputB, 2);


}



void stepExtruder() {
	if (activeMotor == stepPinB && speedB == 0) return;
	if (activeMotor == stepPinA && speedA == 0) return;
	if (activeMotor == stepPinB) {
		if (stepsToTakeB <= 0) {
			doneB = 1;
			speedB = 0;
			return;
		}
		else {
			doneB = 0;
			stepsToTakeB--;
		}
	}
	else if (activeMotor == stepPinA) {
		if (stepsToTakeA <= 0) {
			doneA = 1;
			speedA = 0;
			return;
		}
		else {
			doneA = 0;
			stepsToTakeA--;
		}
	}
	digitalWrite(activeMotor, (int)motorState);
	motorState = !motorState;
}

float readThermistor(uint8_t pin) {

	uint8_t i;
	float average;

	for (i = 0; i < NUMSAMPLES; i++) {
		samples[i] = analogRead(pin);
		delay(1);
	}
	average = 0;
	for (i = 0; i < NUMSAMPLES; i++) {
		average += samples[i];
	}
	average /= NUMSAMPLES;

	average = (1023 / average) - 1;     // (1023/ADC - 1) 
	average = SERIESRESISTOR / average;  // 10K / (1023/ADC - 1)

	float steinhart;
	steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;                 // Invert
	steinhart -= 273.15;                         // convert to C

	return steinhart;
}