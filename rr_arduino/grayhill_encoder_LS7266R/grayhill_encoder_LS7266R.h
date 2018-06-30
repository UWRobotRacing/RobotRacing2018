/************************************************************/
//  Robot Racer encoder library                             //
//  Written By: Tom Meredith, Noah Abradjian, Toni Ogunmade //
//  Last Updated: Dec 16, 2015                              //
/************************************************************/

#ifndef GRAYHILL_ENCODER_LS7266R_H
#define GRAYHILL_ENCODER_LS7266R_H

#include "Arduino.h"
#include <SPI.h>

#define EC_BAUD_RATE		115200 
#define PPR                   128          //512 for full Quad Decoding, 

//speed is m/s
class Encoder {
	private:
		int pin;
		int currentCount;
		bool direction;
		int frequency;
		int countsToMetres;
		
	public: 
		Encoder(int encoderPin, int multiplier, int freq);
		bool updateSpeed(float timeDiff, float &currentSpeed); //gets current speed from pulse count
		void setMultiplier(float newmultiplier); //sets CountstoMetres value
		void switchDirection();
		void setup();
};

#endif
