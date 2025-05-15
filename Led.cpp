#include "Led.h"
#include <Arduino.h>

Led::Led(){}

Led::~Led() {}

void Led::begin(int pin) {
  _pin = pin;
	pinMode(pin,OUTPUT);
}

bool Led::getState() {
	// return state of led
	return (_ledState);
}

void Led::setState(bool state) {
	// switch led on or off
	_ledState = state;
	digitalWrite(_pin,state);
}