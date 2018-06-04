READ ME

@Author: Tsugumi Murata (tsuguminn0401) 
@Note: Code created and tested using Mac, so information here applies only on mac 
	For Windows, AtmelStudio should handle these settings,  
@Competition: IARRC 2018

***** How to Flash the Code ******

Physical Settings:
 
	- Program ArduinoISP example file into Arduino Uno 
		- file -> example -> arduinoISP
	- Configure the SPI setting on ATtiny85 with Arduino Uno (MISO,MOSI,SCK,SS) 
	- Remember to have 10uF capacitor between RESET pin and GND on Arduino (to prevent auto-reset) 
		

For Mac: 
	- Download CrossPack from https://www.obdev.at/products/crosspack/index.html
 	- Clone the project from GitHub 
		- Files you need (in same directory) 
			- main.c
			- usiTwiSlave.c 
			- usiTwiSlave.h 
			- Makefile

		- Makefile setting is configured already 
			- only possible change to make is the programmer
				default is:  “-P /dev/cu.usbmodem1431 ”
					- change this to meet your usb port
					- confirm your usbport using Arduino IDE
						-> tool -> port   
Compile: 
	- open terminal
	- “cd” to project directory including all files 
	- “make flash” 
	- this should generate .hex, .o, .elf 

*Side Note: 
	
- if including separate .h and .c file 

	- on makefile 

		- add YOUR_PROGRAM.o on OBJECTS section 
		- add YOUR_PROGRAM.c on cpp: section 
				


