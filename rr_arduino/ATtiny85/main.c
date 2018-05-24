/** @file main.c
 *  @brief Encoder counter program for ATtiny85
 *
 *  This contains the code that handles the conversion of encorder pulses to count.
 *  Additionally, the ATtiny85 communicates with Arduino Mega through I2C (Attiny is slave),
 *  and when request is sent from the Mega, ATtiny outputs the count
 *
 *  @author Tsugumi Murata (tsuguminn0401)
 *  @competition IARRC 2018
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>                //library for interrupt
#include <stdint.h>                       //library for standard integer
#include "usiTwiSlave.h"                  //I2C lib

#define F_CPU          8000000UL         //set clock frequency to 8MHz
#define SLAVE_ADDRESS  0x07              //salve address
#define DATA_MASK      0x18              //data mask (0b011000)

typedef enum {
    ZERO_ZERO   = 0x00,
    ZERO_ONE    = 0x01,
    ONE_ZERO    = 0x02,
    ONE_ONE     = 0x03
} encoder_state_t;

volatile uint8_t encod_prev  = 0x00;
volatile uint8_t encod_curr  = 0x00;
volatile int16_t encod_count = 0x00;

/** @brief Handles ISR when PB3 or PB4 detects logic change
 *  @param PCINT0_vect Checks for interrupt on PCINT0 (PORTB)
 *  @return Void
 *  @note   increment:
                BA
                00 -> 10
                10 -> 11
                11 -> 01
                01 -> 00
 z
            decrement:
                00 -> 01
                01 -> 11
                11 -> 10
                10 -> 00
 */
ISR(PCINT0_vect){
    cli();  //disable interrupt
    encod_curr = (PINB & DATA_MASK) >> 3;

    switch (encod_prev) {
        
        case (ZERO_ZERO):
            if ( encod_curr == ONE_ZERO  ){
                encod_count++;
            }
            else if ( encod_curr == ZERO_ONE ){
                encod_count--;
            }
            encod_prev = encod_curr;
        sei();
        break;
            
        case (ZERO_ONE):
            if ( encod_curr == ZERO_ZERO ){
                encod_count++;
            }
            else if ( encod_curr == ONE_ONE ){
                encod_count--;
            }
            encod_prev = encod_curr;
        sei();
        break;
        
        case (ONE_ZERO):
            if ( encod_curr == ONE_ONE ){
                encod_count++;
            }
            else if ( encod_curr == ZERO_ZERO ){
                encod_count--;
            }
            encod_prev = encod_curr;
        sei();
        break;
        
        case (ONE_ONE):
            if ( encod_curr == ZERO_ONE ){
                encod_count++;
            }
            else if ( encod_curr == ONE_ZERO ){
                encod_count--;
            }
            encod_prev = encod_curr;
        sei();
        break;
        
    }
}


int main(void)
{
    
//port register
//set PB5 to output
    
    DDRB &= ~(1 << DDB0) | ~(1 << DDB2) | ~(1 << DDB3) | ~(1 << DDB4) ; //set pins to input

//two wire mode [Slave]
    //PB0 input initially (SDA)
    //PB2 input (SCK)
    
//encoder pulses
    //PB3 to input  (PULSE A)
    //PB4 to input  (PULSE B)
    
//Set up the USI communicatin
    usiTwiSlaveInit(SLAVE_ADDRESS);

//initial encoder value
    encod_prev = (PINB & DATA_MASK) >> 3;     //mask the data for PB3 and PB4 and shift 3 bits to right

    
    //interrupt setting
    GIMSK |= (1<< PCIE);
    PCMSK |= (1 << PCINT3) | (1 << PCINT4);
    sei();  //enable the interrupt
    
    
    
    while(1)
    {
        
        //if data received from master
        if(usiTwiDataInReceiveBuffer()){
            
            switch(usiTwiReceiveByte()){
                //trigger signal detected from master
                case 'r':
                    cli();                              //disable interrupt
                    usiTwiTransmitByte(encod_count);    //send encod_count to master
                    PORTB |= ( 1 << PB1);
                
                    //encod_count = 0;                    //reset the count
    
                sei();                                    //enable interrupt
                break;
            }
        }
        
        
    }
    return 0;   /* never reached */
}


