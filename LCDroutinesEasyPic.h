/******************************************************************************
 *
 * Author(s): Gabriel LoDolce, Trudy Schwartz
 * Author of last change: Kent Lee
 * Date of last change: 6/29/2018
 * Revision: 3.0
 *
 *******************************************************************************
 *
 * FileName:        LCDroutinesEasyPic.h
 * Dependencies:    xc.h 
 * Processor:       PIC18F
 * Compiler:        xc8
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the LCD
 ******************************************************************************/

#include <xc.h>

#ifndef _LCD_ROUTINES_B_
#define _LCD_ROUTINES_B_

/*------------------------------------------------------------------------------
 * Definitions for this LCD interface for EasyPic Pro v7
 -----------------------------------------------------------------------------*/

/* RS Pin Assignments */
#define LCD_RS_TRIS     TRISBbits.TRISB4
#define LCD_RS_LAT      LATBbits.LATB4

/* E Pin Assignments */
#define LCD_E_TRIS	TRISBbits.TRISB5
#define LCD_E_LAT	LATBbits.LATB5

/* Data Pin Assignments. Note we only need the upper nibble
** but it is hard to break apart by nibbles. We have to break apart for new LCD since RS and E
** are also on PORTB!!
*/
#define LCD_DATA_TRIS TRISB
#define LCD_DATA_LAT LATB
#define LCD_DATA_PORT PORTB

/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	InitLCD
 *     Parameters:      None
 *     Description:		This function initializes the character LCD.
 *						This function generates a 0.1s and 0.01s delay using the
 *						xc Delay library
 *
 ******************************************************************************/
void InitLCD( void );

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning command. The string must
 *						also be terminated by the null character
 *
 ******************************************************************************/
void DisplayC( const char *LCDStr ); //removed far qualifier

/******************************************************************************
 *     Function Name:	DisplayV
 *     Parameters:      Pointer to a character array in data memory
 *     Description:		This function sends a character array in data memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning command. The string must
 *						also be terminated by the null character
 *
 ******************************************************************************/
void DisplayV( const char *LCDStr );

#endif


