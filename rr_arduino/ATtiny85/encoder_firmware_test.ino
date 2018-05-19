/*
 * @Author      : Tsugumi Murata (tsuguminn0401)
 * @Descrip     : Encoder_firmware I2C test code for ATtiny85 
 *                 Arduino is the Master 
 *                 ATtiny85 is the Slave 
 * 
 * @competition : IARC 2018 
 */


#include <Wire.h> 

#define SLAVE_ADDRESS 0x07 
const char request_message = 'r';
int8_t count_first=0, count_second=0; 
int16_t count = 0; 
int16_t data[32]; 

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Begin counter pulse \n"); 

  Wire.beginTransmission(SLAVE_ADDRESS); 
  Wire.write(request_message); 
  Wire.endTransmission(false);
   
  count = 0; 
  Wire.requestFrom(SLAVE_ADDRESS, 1, true); 
  count_first = Wire.read(); 
  Wire.requestFrom(SLAVE_ADDRESS, 1, true); 
  count_second = Wire.read();
  
  count = (count_first << 8) | count_second; 
  Serial.println(count); 

  delay(1000); 
  

}
