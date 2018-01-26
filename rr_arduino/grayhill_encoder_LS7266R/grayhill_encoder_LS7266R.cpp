/************************************************************/
//  Robot Racer encoder library function definitions        //
//  Written By: Tom Meredith, Noah Abradjian, Toni Ogunmade //
//  Last Updated: Dec 16, 2015                              //
/************************************************************/
#include "Arduino.h"
#include <SPI.h>
#include <grayhill_encoder_LS7266R.h>

Encoder::Encoder(int encoderPin, int multiplier, int freq) { //constructor
	pin = encoderPin;
	countsToMetres = multiplier;
	frequency = freq;
	currentCount = 0;
	direction = true;
}

bool Encoder::updateSpeed(float timeDiff, float &currentSpeed) { //updates speed if necessary, returns whether or not speed was updated to know if previous time should be updated
	if (timeDiff >= 1.0/(float)frequency) { //if time difference is greater than period, update to new speed
		//reads current position (for currentCount)
		digitalWrite(pin,HIGH); 
		digitalWrite(pin,LOW);
		SPI.transfer(0b01100000);
		byte response1 = SPI.transfer (0);
		byte response2 = SPI.transfer (0);
		digitalWrite(pin,HIGH);
		currentCount = ((response1 << 8)|response2) ;
		//resets encoder position reading
		digitalWrite(pin,HIGH);
		digitalWrite(pin,LOW);
		SPI.transfer(0b00100000);
		digitalWrite(pin,HIGH);
		//calculates speed
		currentSpeed = ((float)currentCount/(float)countsToMetres)/timeDiff;
		return true; //need to reset previous time if true
	} 
	else //else, speed is unchanged and no need to update previous time
		return false;
}

void Encoder::setMultiplier(float newmultiplier) { //sets countsToMetres multiplier 
	countsToMetres = newmultiplier;
	return;
}

void Encoder::switchDirection() { //switches the direction boolean if called
	direction != direction;
	return;
}

void Encoder::setup() { //sets up position counter
  pinMode(pin,OUTPUT);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  digitalWrite(pin,HIGH);
  digitalWrite(pin,LOW);
  SPI.transfer(0b00100000); //Write to counter
  digitalWrite (pin,HIGH);

  digitalWrite(pin,HIGH);
  digitalWrite(pin,LOW);
  SPI.transfer(0b10001000); //10001xxx Write to MDR0
  SPI.transfer(0b00000011); //00000011
  digitalWrite (pin,HIGH);

  delay (10);

  digitalWrite(pin,HIGH);
  digitalWrite (pin,LOW); 
  SPI.transfer(0b10010000); //10010xxx Write to MDR1
  SPI.transfer(0b00000010); //00000010
  digitalWrite(pin,HIGH); 
}
