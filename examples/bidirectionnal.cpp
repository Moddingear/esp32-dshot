//Rename this to main.cpp or <something>.ino if you're in Arduino
//Starts the ESC, sets it to be bidirectionnal, then beeps, then makes it go one way then the other over and over at low speeds


#include <Arduino.h>
#include "DShotESC.h"

#define PEAKSPEED 50
#define SINE_DURATION 10000.f //duration of the full cycle, in millis

DShotESC esc0;

void setup()
{
	Serial.begin(115200);
	esc0.install(GPIO_NUM_12, RMT_CHANNEL_0);
	esc0.init();
	esc0.setReversed(false);
	esc0.set3DMode(true);
	for (int i = 0; i < 5; i++)
	{
		esc0.beep(i);
	}
}

void loop() {
	int16_t milliswrap = sin(millis()*2/SINE_DURATION*PI)*PEAKSPEED;
	
	esc0.sendThrottle3D(milliswrap);
	delay(1);
	
}