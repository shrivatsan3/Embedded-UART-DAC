/******************************************************************************
 *
 * Author(s): Gabriel LoDolce, Trudy Schwartz
 * Author of last change: Kent Lee
 * Date of last change: 6/29/2018
 * Revision: 3.0
 * Updated to use XC8 compiler instead of c18
 *******************************************************************************
 * 
 * FileName:        LCDroutinesEasyPic.c
 * Dependencies:    xc.h, string.h, LCDroutinesEasyPic.h
 * Processor:       PIC18F
 * Compiler:        XC8
 *
 *******************************************************************************
 *
 * File Description: Implementation file for the LCD routines library
 *
 ******************************************************************************/


#include <xc.h>
#include <string.h>
#include "LCDroutinesEasyPic.h"

#define _XTAL_FREQ 16000000

// LCD initialization string
static const char LCDInitStr_[] = "\x33\x32\x28\x0C\x01\x06";      
/*  0x33 0x32 "wakes up" the LCD and sets it to 4 bit mode (3,3,3 is wake up. 2 sets 4 bit mode)
 *  0x28 sets it to 2-line mode with 5x8 dot matrix
 *  0x0C turns on display and sets cursor and cursor blinking to off
 *  0x01 clear display
 *  0x06 assigns cursor moving direction
 */


/*------------------------------------------------------------------------------
 * Public functions intended for the user
 -----------------------------------------------------------------------------*/

/********************************************************************
 *     Function Name:   InitLCD 
 *     Parameters:      None 
 *     Description:     This function initializes the LCD by sending the 
 *                      LCDInitStr_ commands. These commands wake up the LCD, 
 *                      set it to 4 bit mode, and configure cursor and display
 *                      settings. 
 *
 ********************************************************************/
void InitLCD(void) {
	unsigned char count = 0;
    unsigned char nibble = 0;
	
    // Delay 40 ms for the LCD controller to come out of reset
	
    __delay_ms(40);
    
	// Drive RS low for command mode
	LCD_RS_LAT = 0;

	// Send Each Byte one nibble at a time until we see the null character
	while( LCDInitStr_[count] != 0x00 ) {
        LCD_DATA_LAT = 0;
		LCD_E_LAT = 1;                          // Drive E high
        nibble = 0xF0 & LCDInitStr_[count];     // Mask to get upper nibble of LCD string
        nibble = nibble>>4;                     // Shift right by 4 to make lower nibble for data ports
        LCD_DATA_LAT = nibble | LCD_DATA_PORT;  // Mask the upper nibble to ONLY last four of PORTB(RB3:0)
        LCD_E_LAT = 0;                          // Drive E low so LCD will process input     
        __delay_ms(10);                         // Delay 10ms
        
        LCD_DATA_LAT = 0;
		LCD_E_LAT = 1;                          // Drive E high
        nibble = 0x0F & LCDInitStr_[count];     // Mask to get lower nibble of LCD string
		LCD_DATA_LAT = nibble | LCD_DATA_PORT;	// Send the lower nibble w/o changing RB4:7
        LCD_E_LAT = 0;                          // Drive E low so LCD will process input
		__delay_ms(10);                         // Delay 10ms
        
		count++;                               
	}
     
	LCD_RS_LAT = 1;                             // Drive RS HIGH to exit command mode
}

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning command. The string must
 *						also be terminated by the null character
 *
 ******************************************************************************/
void DisplayC( const char *LCDStr ) {
    char temp[10];   // Temporary buffer to store input in
    strcpy(temp,LCDStr); // Creating proper data input for DisplayV, xc8 only needs strcpy
    DisplayV(temp);             // Calling DisplayV function
}

/******************************************************************************
 *     Function Name:	DisplayV
 *     Parameters:      Pointer to a character array in data memory
 *     Description:		This function sends a character array in data memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning command. The string must
 *						also be terminated by the null character
 *
 *
 ******************************************************************************/
void DisplayV( const char * LCDStr ) {
    unsigned char count = 0;
    unsigned char nibble = 0;
    
    while(LCDStr[count] != 0x00) {
        // Transmitting the upper nibble
        LCD_E_LAT = 0;
        LCD_DATA_LAT = LCD_DATA_PORT & 0b11000000;
        if(count==0){
            LCD_RS_LAT = 0;     // First hex character is a command - cursor location
        }
        else{  
            LCD_RS_LAT = 1;     // Telling the LCD we are about to transmit DATA!
        }     
        LCD_E_LAT = 1;
        nibble = 0xF0 & LCDStr[count];
        nibble = nibble >> 4;
        LCD_DATA_LAT = nibble | LCD_DATA_PORT;
        LCD_E_LAT = 0;
        __delay_us(50);         // Delay to allow LCD to display char 50 us delay

        // Transmitting the lower nibble
        LCD_DATA_LAT = LCD_DATA_PORT & 0b11000000;
        if(count==0){
            LCD_RS_LAT = 0;     // First hex character is a command - cursor location 
        }
        else{  
            LCD_RS_LAT = 1;     // Telling the LCD we are about to transmit DATA!
        }   
        LCD_E_LAT = 1;
        nibble = 0x0F & LCDStr[count];
        LCD_DATA_LAT = nibble | LCD_DATA_PORT;
        LCD_E_LAT = 0;
        __delay_us(50);        // Delay to allow LCD to display char 50 us delay
        

        count++;
    }
} 

