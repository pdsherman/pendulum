/*
 * File:    main.c
 * Author:  pdsherman
 * Date:    March 2020
 *
 */

// Device configuration settings
#pragma config FOSC = INTRCCLK, MCLRE = OFF, WDTE = OFF, CP = OFF


/***************** Include Files ********************/
#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <pic16f690.h>

/****************** Module Defines ******************/
#define CMD_ZERO 0x01

/***************** Module Variables *****************/
volatile int32_t pos = 0;
volatile PORTBbits_t prev_portb_state;

/*********** Local Function Declarations ************/

void configure_device(void);

/*********** Local Function Definitions ************/

void main(void) {
    
    configure_device();
    
    prev_portb_state = PORTBbits;
    
    while(true) {
    }
    
    return;
}


void configure_device(void)
{   
    //----- Clock -----//
    OSCCONbits.IRCF = 0b111; // 8 MHz for internal oscillator
    
    //----- GPIO -----//
    // 2 pins setup I2C (RB4, RB6)
    // 2 pins GPIO for encoder (RB5, RB7)
    
    // Analog input
    ANSEL  = 0;
    ANSELH = 0;
    
    // Pins to inputs
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB7 = 1;
    
    // Interrupt on change
    IOCBbits.IOCB5 = 1;
    IOCBbits.IOCB7 = 1;
    
    //----- I2C -----//
    
    // Pins must be set as inputs
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB6 = 1;
    
    // Serial Port Control
    SSPCONbits.SSPM  = 0b0110; // I2C slave mode, 7-bit addr
    SSPCONbits.SSPEN = 1;      // Enable Serial Port
    SSPCONbits.CKP   = 1;      // Enable clock
    
    // Setup I2C Address
    SSPADD = 0x28 << 1;

    //----- Interrupts -----//
    PIE1bits.SSPIE = 1; // Enable SSP interrupt
    
    INTCONbits.PEIE = 1;  // Enable peripheral interrupts
    INTCONbits.RABIE = 1; // Enable PORTB change interrupts
    
    // Clear flags before global interrupt enable
    PIR1bits.SSPIF = 0;
    INTCONbits.RABIF = 0;
    
    INTCONbits.GIE = 1;   // Global interrupt enable
}

void interrupt isr(void) 
{
    
    if(INTCONbits.RABIF) {
        //------- Encoder Interrupt Response -------//
        if(prev_portb_state.RB5 != PORTBbits.RB5) {
            // Change on signal A
            if(PORTBbits.RB5) {
                if(PORTBbits.RB7) {
                    pos -= 1;
                } else {
                    pos += 1;
                }
            } else {
                if(PORTBbits.RB7) {
                    pos += 1;
                } else {
                    pos -= 1;
                }
            }
        } else if(prev_portb_state.RB7 != PORTBbits.RB7) {
            // Change on signal B
            if(PORTBbits.RB7) {
                if(PORTBbits.RB5) {
                    pos += 1;
                } else {
                    pos -= 1;
                }
            } else {
                if(PORTBbits.RB5) {
                    pos -= 1;
                } else {
                    pos += 1;
                }
            }
            
        }
        
        prev_portb_state = PORTBbits;
        INTCONbits.RABIF = 0;
    } else if(PIR1bits.SSPIF) {
        //------- I2C Interrupt Response -------//
        
        static int count = 0;
        uint8_t data = SSPBUF;
        
        
        if(SSPSTATbits.READ_WRITE) {
            if(SSPSTATbits.DATA_ADDRESS) {
                count += 1;
            } else {
                count = 0;
            }
            
            if(count < 4) {
                SSPBUF = (uint8_t)(pos >> (count*8));
            } else {
                SSPBUF = 0;
            }
        } else {
            if(SSPSTATbits.DATA_ADDRESS) {
                if(data == CMD_ZERO) {
                    pos = 0;
                }
            }
        }
        
        SSPCONbits.CKP = 1;
        PIR1bits.SSPIF = 0;
    }
}