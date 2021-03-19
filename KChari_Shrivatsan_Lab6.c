
/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: SHRIVATSAN K.  
 * Date  : 2 NOVEMBER 2019
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4519:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5519: Same as ASEN 4519, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4519:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5519: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const char LCDRow1[] = {0x80,0x00,0x00};
char TEMP_STRING[] = {0x80,'T','=',0x00,0x00,'.',0x00,'C',0x00};    // String to display temperature data
char VOLT_STRING[] = {0xC0,'P','T','=',0x00,'.',0x00,0x00,'V',0x00};    //String to display potential data    
char COMBINED_STRING[] = {'T','=',0x00,0x00,'.',0x00,'C',';','P','T','=',0x00,'.',0x00,0x00,'V',0x00};  //String to display both potential and temp data together
char UART_buffer[9]={}; // String to store command from real term
char TEMP_COMMAND[]={'T','E','M','P','\x00'}; //Following 4 strings are compared with UART_buffer to identify the command
char VOLT_COMMAND[]={'P','O','T','\x00'};
char CONT_ON[]={'C','O','N','T','_','O','N','\x00'};
char CONT_OFF[]={'C','O','N','T','_','O','F','F','\x00'};
unsigned int LED_Blink[2] = {5,54};     // 5 -> switch on for 100 ms, 54 -> switch off for 900ms
unsigned int Alive_index = 1;   // toggle between blink LED on and off times
unsigned int Alive_count = 0;   // store count for on and off times
unsigned int one_sec_count = 0; // to keep track of one second; used in continous display on realterm
int one_sec_flag = 0;   // notifies when one second has passed
float VOLT_DATA = 0;    // Variable to store the result of A/D conversion of voltage
float TEMP_DATA = 0;    // Variable to store the result of A/D conversion of temperature   
unsigned int D1 = 0;    // FORMAT FOR TEMPERATURE = D1D2.D3
unsigned int D2 = 0;    // FORMAT FOR VOLTAGE = D1.D2D3
unsigned int D3 = 0;    
unsigned int discard = 0; // Read ADRES and discard first conversion value
unsigned int INDEX = 0; // To receive from Realterm
unsigned int count = 0; // acts as a cursor for character strings
unsigned int temporary = 0; // read on framing error    
unsigned int SPI_read = 0; // discard the value in SSP1BUF    
int DATA_DAC = 0x3000; // data to be sent to DAC   
unsigned int RAMP = 0; // Variable to create positive or negative slope for triangular wave   
unsigned int INCREMENT_COUNT = 0; // Variable to count DAC Steps   

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void Initialize_USART(void);    // Function to initialize USART
void Initialize_AD(void);       // Function to initialize A/D Converter
void TMR1handler(void);     // Interrupt handler for TMR1    
void TMR0handler(void);     // Interrupt handler for TMR0
void USART_Receiver_handler(void); // Interrupt handler for USART reception
void Send_Data_USART(char);     // Function to send data over USART
void UART_TRANSMIT(void);           // Transmit to Realterm
void Read_volt(void);       // Read voltage from potentiometer
void display_volt(void);    // LCD voltage reading display
void Read_temp(void);       // Read temperature from temperature sensor
void display_temp(void);    // LCD temperature reading display
void Initialize_SPI(void);      // Function to initialize SPI
void send_SPI(void);            // Transmit to DAC
//void ADhandler(void);       // Interrupt handler for A/D conversion


/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
     
     while(1) {
     
         Read_volt();           // Read potential and display to LCD
         Read_temp();           // Read temperature and display to LCD    
         UART_TRANSMIT();       // Transmit data to Realterm
         send_SPI();
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/

void Initial() {
    // Configure the IO ports
    TRISD  = 0b00001111;    // Configure bit 7-4 as output
    LATD = 0;               // Clear output Latch
    TRISC  = 0b1000000;    // Set RC7 as input and RC6 as output
    LATC = 0;
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero


    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    
    LATDbits.LATD5 = 1;         // Switch ON RD5
    __delay_ms(500);            // Wait 0.5 sec
    LATDbits.LATD5 = 0;         // Switch OFF RD5
    
    LATDbits.LATD6 = 1;         // Switch ON RD6         
    __delay_ms(500);            // Wait 0.5 sec
    LATDbits.LATD6 = 0;         // Switch OFF RD6
    
    LATDbits.LATD7 = 1;         // Switch ON RD7
    __delay_ms(500);            // Wait 0.5 sec
    LATDbits.LATD7 = 0;         // Switch OFF RD7
    
            
    // Initializing TMR0
    T0CON = 0b00001000;             // 16-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;

    // Initializing TMR1
    T1CON = 0b00000000;             // 16-bit, Fosc / 4, no pre/post scale timer
    TMR1L = 0;                      // Clearing TMR1 registers
    TMR1H = 0;

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    IPR1bits.TMR1IP = 0;            // Assign low priority to TMR1 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    PIE1bits.TMR1IE = 1;            // Enable TMR1 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    T1CONbits.TMR1ON = 1;           // Switch on TMR1
    Alive_count = LED_Blink[Alive_index];   // initialize alive count for 100ms delay count
    
    // Initializing A/D Converter
    Initialize_AD();
    
    // Initializing USART
    Initialize_USART();  
    
    // Initializing SPI
    Initialize_SPI();  
    
    
}
    
void Initialize_SPI(){
    
     TRISCbits.TRISC5 = 0; // Data Out -> o/p
     TRISCbits.TRISC4 = 1; // Data in -> i/p
     TRISCbits.TRISC3 = 0; // Clock -> o/p 
     TRISEbits.TRISE0 = 0; // Chip Select -> o/p
     TRISAbits.TRISA5 = 1; // Set RA5 to input
     
     
     SSP1STAT = 0b01000000; //SSP1STATBITS.CKE = 1, Sample at the middle

     SSP1CON1 = 0b00100000; //SSP1CON1BITS.CKP = 0;
                            //SSP1CON1BITS.SSPEN = 1;
    }

void Initialize_USART(){
    
    RCSTA1bits.SPEN = 1;        // Enable serial port  
    TXSTA1 = 0b00000100;        // 8-bit asynchronous transmission; high speed baud rate mode  
    BAUDCON1bits.BRG16 = 0;     // 8-bit baud rate generator
    SPBRG = 51;                 // SPBRG count for Baud rate of 19200
    PIE1bits.RC1IE = 1;         // Enable interrupt on reception
    IPR1bits.RC1IP = 0;         // Give low priority
    TXSTA1bits.TXEN = 1;        // Enable transmission 
    RCSTA1bits.CREN = 1;        // Enable continuous reception
}

void Initialize_AD(){
    
//    IPR1bits.ADIP = 0;               // Assign low priority to A/D interrupt
//    PIE1bits.ADIE = 1;              // Enable A/D interrupts
        
    TRISA =     0b00001001;   //Set RA0 and RA3 as input since potentiometer is connected to it
    ADCON0 =    0b00000001;        //ADCON0 set for AN0, A/D ON
    ADCON1 =    0b00000000;   //Set Vref+ to VDD and Vref- to GND 
    ADCON2 =    0b10010101;   //Set Acquisition time to 4*TAD and TAD equal to 16*TOSC
    ANCON0 =    0b00001001;   //Set Bit0 and Bit4 to have analog input on RA0 and RA3 
    ANCON1 =    0b00000000;   //Set all other pins to digital I/O
    ANCON2 =    0b00000000;    
    __delay_us(2);              //Acquisition delay
    ADCON0bits.GO = 1;          //ADC conversion begins
      
}


void send_SPI(){
    
    INCREMENT_COUNT++; // Mainline runs for 7.15 ms. 7.15*140 = 1 sec
    if(INCREMENT_COUNT == 140){
    INCREMENT_COUNT=0; 
    RAMP ^=1;       //Ramp = 0 -> Positive slope. Ramp = 1 -> Negative slope
    }
    
    if(RAMP == 0){DATA_DAC = DATA_DAC + 0x001D;}  // 4096/140 = 0x001D
    if(RAMP == 1){DATA_DAC = DATA_DAC - 0x001D;}
    
    LATEbits.LATE0 = 0;     //Enable DAC by driving Chip Select low
    
    SSP1BUF = DATA_DAC>>8;      //Send Higher Byte first
    //SSP1BUF = 0x30;
            
    while(SSP1STATbits.BF == 0){}    // Wait for Buffer Flag to set
    
    SPI_read = SSP1BUF;     //Read Buffer
    
    SSP1BUF = DATA_DAC;     //Send lower Byte
    //SSP1BUF = 0x00;
    
    while(SSP1STATbits.BF == 0){}// Wait for Buffer Flag to set
    
    SPI_read = SSP1BUF;     //Read Buffer
    
    LATEbits.LATE0 = 1;     //Drive Chip Select High
    }


void Send_Data_USART(char ascii){
    while(PIR1bits.TX1IF==0){}  // Wait for register to become empty
    TXREG1 = ascii;             // Write into register    
    (void)0;                    // Don't read the flag immediately
    }

void UART_TRANSMIT(){
    
if ((strcmp(UART_buffer,VOLT_COMMAND)) == 0){       // Realterm transmitted 'POT'
    memset(UART_buffer, 0, sizeof(UART_buffer));    // Clear UART_Buffer so that string comparison results in false on next iteration
    INDEX = 0;      // reset index for next reception
    count = 4;
    while(VOLT_STRING[count] != 0x00){
        Send_Data_USART(VOLT_STRING[count]);           //Send voltage data one byte at a time 
        count++;
    }
    Send_Data_USART(0x0A);          // Send line feed in the end
        
}
else if ((strcmp(UART_buffer,TEMP_COMMAND)) == 0){  // Realterm transmitted 'TEMP'
   
    memset(UART_buffer, 0, sizeof(UART_buffer));    // Clear UART_Buffer so that string comparison results in false on next iteration
    INDEX = 0;      // reset index for next reception
    count = 3;
    while(TEMP_STRING[count] != 0x00){
        Send_Data_USART(TEMP_STRING[count]);            //Send temperature data one byte at a time
        count++;
    }
    Send_Data_USART(0x0A);              // Send line feed in the end

}    
else if ((strcmp(UART_buffer,CONT_ON)) == 0){       // Realterm transmitted 'CONT_ON'
    INDEX = 0;          // reset index for next reception
    count = 0;
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    
    if(one_sec_flag){               // if one second has passed
    one_sec_flag = 0;    
    T0CONbits.TMR0ON = 0;           // Turning off TMR0
    
    while(COMBINED_STRING[count] != 0x00){
        Send_Data_USART(COMBINED_STRING[count]);        //Send temperature and voltage data
        count++;
        }    
    Send_Data_USART(0x0A);          // Send line feed in the end
    }
    
    }


else if ((strcmp(UART_buffer,CONT_OFF)) == 0){      // Realterm transmitted 'CONT_OFF'
    memset(UART_buffer, 0, sizeof(UART_buffer));    // Clear UART_Buffer so that "CONT_ON" string comparison results in false on next iteration
    INDEX = 0;              // reset index for next reception
    }    
}

void Read_volt(){
    
    ADCON0 = 0b00000001;        //ADCON0 set for AN0, A/D ON
    __delay_us(2);
    ADCON0bits.GO = 1;          //ADC conversion begins
    while(ADCON0bits.GO){}  // used to discard the first ADC conversion value (OUTPUT) which is not correct
    discard = ADRES;

    ADCON0bits.GO = 1;          // start conversion
    while(ADCON0bits.GO){}  
    VOLT_DATA = ((ADRES+0x23)*3.3)/4096.0;        // 3.3 V in 4096 steps 
    display_volt();       // Call LCD routine        
}

void display_volt(){
    D1 = VOLT_DATA;     // Number in unit's place
    D2 = (VOLT_DATA-D1)*10;         //Number in first decimal place
    D3 = (((VOLT_DATA-D1)*10)-D2)*10;       //Number in second decimal place
    VOLT_STRING[4] = D1 + 0x30;     //Update Voltage string
    VOLT_STRING[6] = D2 + 0x30;
    VOLT_STRING[7] = D3 + 0x30;
    COMBINED_STRING[11] = D1 + 0x30;    // Update Combined string 
    COMBINED_STRING[13] = D2 + 0x30;
    COMBINED_STRING[14] = D3 + 0x30;
    
    DisplayV(VOLT_STRING);      //Display to LCD
 }


void Read_temp(){
    ADCON0 = 0b00001101;        //ADCON0 set for AN3, A/D ON
    __delay_us(2);
    ADCON0bits.GO = 1;          //start ADC conversion
    while(ADCON0bits.GO){}   // discard the first conversion value
    discard = ADRES;
    __delay_us(2);
    ADCON0bits.GO = 1;          
    while(ADCON0bits.GO == 1){}   
    TEMP_DATA = (ADRES*3.3)*100.0/4096.0;
    display_temp();             // call LCD routine 
}

void display_temp(){  
    D1 = (TEMP_DATA)/10;        //Number in tenth's place
    D2 = ((TEMP_DATA/10)-D1)*10;        // Number in Unit's place
    D3 = ((((TEMP_DATA/10)-D1)*10)-D2)*10;      //Number in First decimal place
    TEMP_STRING[3] = D1 + 0x30;     //Update temperature display
    TEMP_STRING[4] = D2 + 0x30;
    TEMP_STRING[6] = D3 + 0x30;
    COMBINED_STRING[2] = D1 + 0x30;     //Update combined string
    COMBINED_STRING[3] = D2 + 0x30;
    COMBINED_STRING[5] = D3 + 0x30;
    
    DisplayV(TEMP_STRING); // display temperature data to LCD
}
    
/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
                
        if( PIR1bits.TMR1IF ) {     //Check TIMER1 interrupt
            TMR1handler();
            continue;
        }
        
        if ( PIR1bits.RC1IF ) {     //Check USART reception interrupt
            USART_Receiver_handler();
            continue;
        }
                
        if( INTCONbits.TMR0IF ) {   //Check TIMER0 interrupt
            TMR0handler();
            continue;
        }
                
//        if( PIR1bits.ADIF ) {
//          ADhandler();
//          continue;
//        }
               
        // Save temp copies of WREG, STATUS and BSR if needed.
             
        break;      // Supports RETFIE automatically
    }
}


/******************************************************************************
 * TMR1handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/

void TMR1handler() {
    if( Alive_count > 0 ) { Alive_count--; }       
    else {
        LATDbits.LATD4 ^= 1;        //Toggle LATD4
        Alive_index ^= 1;           //Toggle index
        Alive_count = LED_Blink[Alive_index];      //Toggle the count, 100ms for ON 900ms for OFF
    }
    PIR1bits.TMR1IF = 0;      //Clear flag and return to polling routine
}

void TMR0handler() {
    if( one_sec_count < 60 ) { one_sec_count++; }       //Check if one second has passed since last CONT_ON update 
    else {
        one_sec_flag = 1;           
        one_sec_count = 0;
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}

void USART_Receiver_handler(){
    if(RCSTA1bits.FERR == 1){       //Check for framing error
        temporary = RCREG1;
        return;   }
    
    if(RCSTA1bits.OERR == 1){       //Check for overrun error
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
        return;    }
    
    UART_buffer[INDEX]= RCREG1;      //Increment INDEX for next character reception
    INDEX++;
    UART_buffer[INDEX]=0x00;         //Add NULL character
    
    if (INDEX > 8){             //Reset index when a larger string is received    
        INDEX = 0;
    }
    
    PIR1bits.RC1IF = 0;         // Clear reception flag
    
    }

//void ADhandler() {
////  
//    skip_first_conv -= 1 ;
////            
//    if(skip_first_conv==0){
//        skip_first_conv = 2;
//        if(ADCON_INDEX == 0){
//            VOLT_DATA = ((ADRES+0x23)*3.3)/4096.0;
//        }
//        else{
//            TEMP_DATA = (ADRES*3.3)*100.0/4096.0;
//        }
//        
//        ADCON_INDEX ^= 1;
//        ADCON0 = ADCON_TOGGLE[ADCON_INDEX];
//
//    }
//    else{
//        discard = ADRES;
//        }
//    __delay_us(2);
//    ADCON0bits.GO = 1;
//    PIR1bits.ADIF = 0;      //Clear flag and return to polling routine
//}
