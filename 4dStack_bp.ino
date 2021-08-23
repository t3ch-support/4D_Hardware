/*
    Name:       4dStack.ino
    Created:	12-Mar-19 11:24:14 AM
    Author:     Tech Support
*/


#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

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

const int stepPinA = 6;
const int stepPinB = 11;
const int dirPinA = 5;
const int dirPinB = 10;

void setup()
{
	lcd.begin(20, 4);
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	lcd.setBacklight(HIGH);
	lcd.home();
	lcd.print("4D-Stack");

	pinMode(stepPinA, OUTPUT);
	pinMode(stepPinB, OUTPUT);
	pinMode(dirPinA, OUTPUT);
	pinMode(dirPinB, OUTPUT);


	Serial.begin(9600);
}

void loop()
{
	lcd.setCursor(0, 1); // go to start of 2nd line

	lcd.print("HIGH");
	digitalWrite(dirPinA, HIGH);
	for (int x = 0; x < 200; x++) {
		digitalWrite(stepPinA, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPinA, LOW);
		delayMicroseconds(500);
	}
	delay(1000);
	lcd.setCursor(0, 1); // go to start of 2nd line
	lcd.print("LOW");
	digitalWrite(dirPinA, LOW);
	for (int x = 0; x < 200; x++) {
		digitalWrite(stepPinA, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPinA, LOW);
		delayMicroseconds(500);
	}
	delay(1000);
	
}