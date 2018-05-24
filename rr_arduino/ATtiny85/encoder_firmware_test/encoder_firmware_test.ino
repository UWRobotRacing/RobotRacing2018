/*
 * @Author      : Tsugumi Murata (tsuguminn0401)
 * @brief       : Encoder_firmware I2C test code for ATtiny85 
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

　//Send request to slave (ATtiny85)
  //begins the I2C communication 
  Wire.beginTransmission(SLAVE_ADDRESS); 
  Wire.write(request_message); 
  Wire.endTransmission(false);

  //resets count 
  count = 0; 
  //data receive mode 
  Wire.requestFrom(SLAVE_ADDRESS, 1, true); 
  //reads the first 8 bit sent  
  count_first = Wire.read(); 
  //data receive mode 
  Wire.requestFrom(SLAVE_ADDRESS, 1, true); 
  //reads the first 8 bit sent
  count_second = Wire.read();
  
  //shifts the count_first 8 bit to left, and bitwise or with count_second
  count = (count_first << 8) | count_second; 
  Serial.println(count); 

  delay(1000); 
  

}
