#pragma config OSC    = HS     // HS oscillator
#pragma config WDT    = OFF    // Watcdog timer off
#pragma config MCLRE  = ON     // Master clear reset On
#pragma config DEBUG  = OFF    // Debug off
#pragma config LVP    = OFF    // Low voltage programming off
//==========================================================================
#include <p18f4520.h>   // Header files of PIC18F4520
 #include <timers.h>            // Include the Timer Library        
#include <delays.h>     // Include delays
#include <adc.h>
#include <stdlib.h>
#include <mod_xlcd.h>	                // LCD header files some (moddified)    */  
#include <usart.h>			 // Add USART functions
//===========================================================================
//LCD 
#define CLR_LCD      1			    // Clear LCD command code               */
#define HOME_LCD     2			    // Cursor home command code             */
#define LINE1_LCD 0x80              // Position cursor on line 1 command    */
#define LINE2_LCD 0xC0  	        // Position cursor on line 2 command    */
#define CURS_OFF  0x0C		        // Turn Cursor off                      */
#define CURS_L    0x10			    // Move Cursor left                     */
#define CURS_R    0x14			    // Move Cursor right                    */
#define SHIFT_L   0x18		        // Scroll display left                  */
#define SHIFT_R   0x1C		        // Scroll display right

#define BUZ1 PORTCbits.RC3
#define D3 PORTAbits.RA1
#define D4 PORTAbits.RA2
#define D5 PORTAbits.RA5
#define solenoid PORTCbits.RC5
//==========================================================================
void ADCvalue(void);
void test_isr(void);   //
void INT0_ISR(void) ;   // Prototype for the goto that follows
void test_isr0 (void); //for timer 0
void TMR0_ISR(void) ; // for timer 0
void emergencyRoutine(void);

void msg2(void); // should say temperature on line 2
void VirtTermMode(void);
void clearLine2(void);

void keyboard(void);

void UserInput(void);
void checkPin(void);
void readpassword(void);
void unlocked(void);
void wrong(void);

//===============================================
unsigned int result, temperature;      //temperature needs to be converted
unsigned int c, i, x, pin[4];             //k for buzzard


unsigned int KEYBUFF[8], Digit[3];  //3 digit pin and 1 hashtag
unsigned char ASCIIDigit0, ASCIIDigit1, ASCIIDigit2;  
unsigned char ASCIIkeyDigit[7], p;    //ASCHIIIkeyDigit7 is used in USART   p for usart
unsigned char msg_1[]="Temperature:"; // 
unsigned char msg_2[]="C"; // 
unsigned int_extract, rem1, dig1, dig2;
char buf = 0x30;
char ASCIIdig1, ASCIIdig2;
#unsigned char data;

///////////////////////////////////////////////////////////////

void password(void)
{   
   //=====================EPROM write function=====================
   
    {
    EEADR = 0x00;                   /* set EEPROM Address to 0  */
    EEDATA = 0x01;                 // Set EEPROM DATA to write to address to be 0x46
    
    EECON1bits.EEPGD = 0;              // clear EEPGD bit to access the EEPROM memory (rather than the Program memory)
    EECON1bits.CFGS = 0;              //Clear CFGS bit access FLash program or data EEPROM memory not configuration registers
    EECON1bits.WREN = 1;             //Set Write Enable but enable writes
    INTCONbits.GIE = 0;                     // Disable global interrupt
    EECON2 = 0x55;                           //two bytes are written into EECON2 to perform the write operation
    EECON2 = 0xAA;
    EECON1bits.WR = 1;                    //Enable writes by setting the WR bit in EECON1
    
    while(EECON1bits.WR);          //Wait for the write to complete by checking the state of that the WR bit is clear
    
    
    //=====================
        EEADR = 0x01;                   /* set EEPROM Address to 0  */
    EEDATA = 0x02;                 // Set EEPROM DATA to write to address to be 0x46
    
    EECON1bits.EEPGD = 0;              // clear EEPGD bit to access the EEPROM memory (rather than the Program memory)
    EECON1bits.CFGS = 0;              //Clear CFGS bit access FLash program or data EEPROM memory not configuration registers
    EECON1bits.WREN = 1;             //Set Write Enable but enable writes
    INTCONbits.GIE = 0;                     // Disable global interrupt
    EECON2 = 0x55;                           //two bytes are written into EECON2 to perform the write operation
    EECON2 = 0xAA;
    EECON1bits.WR = 1;                    //Enable writes by setting the WR bit in EECON1
    
    while(EECON1bits.WR);          //Wait for the write to complete by checking the state of that the WR bit is clear
    
    //=============================
        EEADR = 0x02;                   /* set EEPROM Address to 0  */
    EEDATA = 0x03;                 // Set EEPROM DATA to write to address to be 0x46
    
    EECON1bits.EEPGD = 0;              // clear EEPGD bit to access the EEPROM memory (rather than the Program memory)
    EECON1bits.CFGS = 0;              //Clear CFGS bit access FLash program or data EEPROM memory not configuration registers
    EECON1bits.WREN = 1;             //Set Write Enable but enable writes
    INTCONbits.GIE = 0;                     // Disable global interrupt
    EECON2 = 0x55;                           //two bytes are written into EECON2 to perform the write operation
    EECON2 = 0xAA;
    EECON1bits.WR = 1;                    //Enable writes by setting the WR bit in EECON1
    
    while(EECON1bits.WR);          //Wait for the write to complete by checking the state of that the WR bit is clear
        
//========================    
    EECON1bits.WREN=0;                    //on completion disable further write operations by clearing the WREN bit
    INTCONbits.GIE = 1;                              //re-enable global interrupt
    }
}

void readpassword(void)
{
    //===============================READTTHE DATA=============  3 password so dig0 dig1 dig2
           {
        EEADR = 0x00;                          // Set EEPROM address to read the data from to  location 0
        EECON1bits.EEPGD = 0;              // Clear EEPGS bit to access the EEPROM Memory (Rather than program memory)
        EECON1bits.CFGS = 0;                    //CLear CFGS but access flash program or data EEPROM memory (not configuration register)
        EECON1bits.RD = 1;                    //CLear Read operations by setting the RD bit in EECON1
        Digit[0] = EEDATA;                   //Show EEPROM data on the LEDS on PORTD
        ASCIIDigit0 = Digit[0] + 0x30;
	
    }
    
    {
        EEADR = 0x01;                          // Set EEPROM address to read the data from to  location 01
        EECON1bits.EEPGD = 0;              // Clear EEPGS bit to access the EEPROM Memory (Rather than program memory)
        EECON1bits.CFGS = 0;                    //CLear CFGS but access flash program or data EEPROM memory (not configuration register)
        EECON1bits.RD = 1;                    //CLear Read operations by setting the RD bit in EECON1
        Digit[1] = EEDATA;                   //Show EEPROM data on the LEDS on PORTD
        ASCIIDigit1 = Digit[1] + 0x30;
    }
    
    {
        EEADR = 0x02;                          // Set EEPROM address to read the data from to  location 02
        EECON1bits.EEPGD = 0;              // Clear EEPGS bit to access the EEPROM Memory (Rather than program memory)
        EECON1bits.CFGS = 0;                    //CLear CFGS but access flash program or data EEPROM memory (not configuration register)
        EECON1bits.RD = 1;                    //CLear Read operations by setting the RD bit in EECON1
        Digit[2] = EEDATA;                   //Show EEPROM data on the LEDS on PORTD
        ASCIIDigit2 = Digit[2] + 0x30;
    }

}


//==========================================================================
/* 
   Following a pragma code directive, all code will be placed in 
   the specified address location till another pragma code directive 
   encountered. for example in this case, that High Interrupt vector is stored in @ 0x08
*/
/////////////Rx interrupt, set to be high priority
#pragma code my_interrupt = 0x08    // "my_interrupt" function is placed on the high priority vector
void my_interrupt(void)           
{                        // In this function we are doing "code mixing" 
	_asm                 // We are telling the compiler that the next instruction will be in assembly
	GOTO test_isr        // Use the GOTO assembly instruction to branch to "test_isr" (line 40) unconditionally
	_endasm              // Tell the compiler that we are not going to use assembly instruction any more
}

#pragma code                       // used to allow linker to locate remaining code
#pragma interrupt test_isr         // Specify a function name and interrupt specifications 
                                   // through the #pragma directive e.g. #pragma interrupt  interrupt-handler-name
void test_isr(void)
{


    if(PIR1bits.RCIF == 1)          // Was the interrupt caused by Rx; if   */
    VirtTermMode();



}



////////////////////////////////////////////////////////
////////////////////timer0 interrupt
#pragma code my_interrupt0 = 0x18
void my_interrupt0 (void) // 
 {
_asm 
GOTO test_isr0//
_endasm
 }
#pragma code // 


#pragma interrupt test_isr0     
	 void test_isr0 (void)    
	 {  if (INTCONbits.TMR0IF == 1)   // Was interrupt caused by Timer 0? 
         { 
         TMR0_ISR();                // Yes , execute TMR0 ISR program
         }  
      }   

 void TMR0_ISR(void)  // for timer 0
         { 
             INTCONbits.TMR0IE = 0;     // Disable any interrupt within interrupts  
	         c=c-1;                     //
               ADCvalue();
     
              if (c==0)//5 second lapsed
            {
	    c=120;// for next 5 sec   .. you need to change it check every 1 minute
             ADCvalue();
	     msg2();
	     clearLine2();

	     
	    }
	    
			 
             WriteTimer0 (65535-62500);  // for 2.5sc requires counting 0 to 48828
             INTCONbits.TMR0IE = 1;     // Re-enable TMR0 interrupts
             INTCONbits.TMR0IF = 0;     // Clear TMR0 flag before returning to main 
         }     
//==========================================================================
void main (void)
{
//////////////////////////////Change port
        TRISA = 0x09; //for ADC input analogie and vref
	LATA = 0x00;
        TRISB = 0x01; //change when keyboard is placed this is for interrupt
	LATB = 0x00;
	ADCON1 = 0x0E;                // Make sure port B pins are digital except pin 1 of port A
	TRISD  = 0x00;                // Port D is configured as output  // only for lcd
	LATD   = 0x00;                // Port D is latched to 0x00
	TRISC  = 0x80;   //for Rx as input
        LATC = 0x00;
	
//////////////////////////////////////////////////// tests
 readpassword();
    ///////////////////////////////////////////////////////////////////////////////
x = 0;


///////////////////////////////////////////////////////////////////////////////PIN INTERRUPT

  
//////////////////////////////////////////////////////////////////////////////////////
 CloseTimer0 ();  // Close The Timer 0  if it was previously Open
    OpenTimer0 (TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8 );  // Configure the Timer0 with Interrupt, ON, 86bits internal clock source Prescaler of 1:8
	c=120;
    WriteTimer0(65535-62500);//5-46875);  // for 1 min it requires 20 times counting to 46875
    INTCONbits.TMR0IF = 0;            // Clear the Timer 0Interrupt Flag   
    INTCONbits.TMR0IE = 1;            // Enable Timer 0Interrupt  
    INTCON2bits.TMR0IP= 0;//low priority bit for timer0  
    
//////////////////////////////////////////////////////////////
      // Timer0 Functions as set in the following operational mode
    OpenTimer1 (  // Open Timer0 with the following settings    
    TIMER_INT_OFF &  // Disable Timer0 interrupts    
    T1_16BIT_RW &   // 16-bit configuration of the timer     
    T1_SOURCE_INT &  // use the internal clock     
    T1_PS_1_8   // divide the internal 5MHz clock by 256   
    );    
///////////////////////////////////////////////////////////////
    RCONbits.IPEN=1;// INT priority bit on
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
    //////////////////////////////////////////////////////////////////////////////LCD 
    
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 25); 	            
    putrsUSART(" Enter pin: \r\n");    // Send the string of characters
    while(BusyUSART());
    
    OpenXLCD(FOUR_BIT & LINES_5X7);	// Use 4 bit Data, 5x7 pixel per char.  */
    




    while(BusyXLCD());	            // Wait for LCD to finish               */
    WriteCmdXLCD(CURS_OFF);        // Turn cursor ON                       */
    while(BusyXLCD());              // Wait for LCD to finish processing    */
    WriteCmdXLCD(SHIFT_DISP_LEFT);  // Shift Cursor Display Left            */
/////////////////////////////////////////////////////////////////Write youre code from here:
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	while(BusyXLCD());      // Wait for LCD to finish processing  
	putrsXLCD("ENTER PIN:                     ");
        while(BusyXLCD());      // Wait for LCD to finish processing  
  	
	  Delay10KTCYx(100);	 
	 Delay10KTCYx(100);
         UserInput();

	 while(1)
	 { 
	 	  Delay10KTCYx(50);	
}		
}
////////////////////////////////////////////////////////////////////////////////////////////
void UserInput(void)
{
//////////////////////////////////////////////////////////////////////

	 	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF      
        putrsXLCD("                                 ");
        while(BusyXLCD());      // Wait for LCD to finish processing 
        i = 0;
	 keyboard();
	 
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	while(BusyXLCD());      // Wait for LCD to finish processing         
	  WriteDataXLCD(ASCIIkeyDigit[i]);
	  while(BusyXLCD());      // Wait for LCD to finish processing  
          Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  
	  
          putrsXLCD("*");
          while(BusyXLCD());      // Wait for LCD to finish processing
	  if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
 
///////////////////////////////////////////////////////////////////////
          i = 1;
         keyboard();
        
	  WriteDataXLCD(ASCIIkeyDigit[i]);
	  while(BusyXLCD());      // Wait for LCD to finish processing  
          Delay10KTCYx(50);	
	  WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	  while(BusyXLCD());      // Wait for LCD to finish processing  	  
          putrsXLCD("**");
          while(BusyXLCD());      // Wait for LCD to finish processing     
	  
	  if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////	      
          i = 2;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("***");
         while(BusyXLCD());      // Wait for LCD to finish processing 
	 	 
          if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
	 
/////////////////////////////////////////////////////////////////////////
         i = 3;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("****");
         while(BusyXLCD());      // Wait for LCD to finish processing 
	 
	 if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////
         i = 4;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("*****");
         while(BusyXLCD());      // Wait for LCD to finish processing 
	 
	  if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////
         i = 5;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("******");
         while(BusyXLCD());      // Wait for LCD to finish processing 
	 
	 if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////
         i = 6;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("*******");
         while(BusyXLCD()) ;      // Wait for LCD to finish processing 
	 
	 if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////
         i = 7;
	 keyboard();
	 
	 WriteDataXLCD(ASCIIkeyDigit[i]);
	 while(BusyXLCD());      // Wait for LCD to finish processing  
         Delay10KTCYx(50);	
	 WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      
	 while(BusyXLCD());      // Wait for LCD to finish processing  	  
         putrsXLCD("********");
         while(BusyXLCD());      // Wait for LCD to finish processing 
	 
	 if (ASCIIkeyDigit[i] == 0x23)
	  {
	  checkPin();
	  }
/////////////////////////////////////////////////////////////////////////

	 
}
void checkPin(void)
{
if (((KEYBUFF[0]) == (Digit[0])) && ((KEYBUFF[1]) == (Digit[1])) && ((KEYBUFF[2]) == (Digit[2])) && ((KEYBUFF[3]) == 23))
 {
 unlocked();
}
else {wrong();}
}



void wrong(void)
{
        x = x + 1;
	if (x == 1)
	{                           // write here LEDs and buzzer
	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("ATTEMPT1          ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	Delay10KTCYx(100);
	D3 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
	}
	
	
	if (x == 2)
	{
	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("ATTEMPT2          ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	Delay10KTCYx(100);
	D3 = 1;
	D4 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	D4 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
	}
	
	if (x == 3)
	{
		WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("ATTEMPT3          ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
		Delay10KTCYx(100);
	D3 = 1;
	D4 = 1;
	D5 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	D5 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	D4 = 1;
	D5 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	D5 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
	
	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("LOCKING KEYPAD     ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	Delay10KTCYx(100);
	Delay10KTCYx(100);
	
	main();
	}
	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("                            ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	UserInput();
}
	

void unlocked(void)
{
         TRISC = 0x00;
	 LATC = 0x00;

	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("UNLOCKING..          ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	Delay10KTCYx(100);

	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("                              ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	 LATC = 0x20;
	 Delay10KTCYx(100);
	main();
}



void msg2(void)
{
       	

	WriteCmdXLCD(LINE2_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
       putsXLCD(msg_1);
       WriteDataXLCD(ASCIIdig1);       
       WriteDataXLCD(ASCIIdig2);       
       putsXLCD(msg_2);
	while(BusyXLCD());      // Wait for LCD to finish processing    */
       Delay10KTCYx(50);
      
}

void clearLine2(void)
{
	WriteCmdXLCD(LINE2_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing  
	putrsXLCD("                             ");	
	while(BusyXLCD());      // Wait for LCD to finish processing    */
}
			 
void ADCvalue(void)
{
	 	OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_20_TAD,ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_EXT & ADC_VREFMINUS_VSS, 0b1110);
		ConvertADC( );                     /* start A/D conversion */
		while(BusyADC( ));                 /* wait for completion */
		
		result = ReadADC();               // Read ADC works only for RIGHT (ADC_RIGHT_JUST)
		CloseADC();
		
                result = result / 10.23;         // Convert Vin to degrees Celsius
		result = result * 1;            // conversion from float type into int type
		////////////////////////////
		if (result == 100)
		{
		emergencyRoutine();
		}
		//////////////////////////
		
		int_extract = result;
	         dig1 = int_extract / 10;         // Extract the first digit of the temperature  - e.g. 23 / 10 = 2
	         rem1 = int_extract % 10;         // Extract the two remainig digits - e.g. 23 % 10 = 3 
	         dig2 = rem1 / 1;         // Extract the second digit of the temperature - e.g. 3 / 1 = 3
	         
		 ASCIIdig1 = dig1 + 0x30; 	// convert BCD to ASCII
		 ASCIIdig2 = dig2 + 0x30;	// convert BCD to ASCII

}

void emergencyRoutine(void)  /// write emergency routine here
{
         solenoid = 1;
        WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing  
	putrsXLCD("EMERGENCY T=100!");	
	while(BusyXLCD());      // Wait for LCD to finish processing    */
 
	WriteCmdXLCD(LINE2_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing  
	putrsXLCD("UNLOCKING... *     *    ");	
	////////////////////////////////////////////////
	      BUZ1 = 0x01;
	      
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      	      BUZ1 = 0x01;
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      	      BUZ1 = 0x01;
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      	      BUZ1 = 0x01;
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      	      BUZ1 = 0x01;
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      	      BUZ1 = 0x01;
     WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
      BUZ1 = 0x00;
      WriteTimer1(0);   // reset the timer to zero    
      while (ReadTimer1() < 62500);     // wait for 1 sec overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
         WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow 
       WriteTimer1(0);   // Reset the timer to zero
      while (ReadTimer1() < 62500);  // wait for 1 sec for overflow
      
      solenoid = 0;
      //////////////////////////////////////////////////////////////////
              	WriteCmdXLCD(LINE1_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing    */
	putrsXLCD("                              ");       //
	while(BusyXLCD());      // Wait for LCD to finish processing    */
		WriteCmdXLCD(LINE2_LCD);// Turn cursor OFF                      */
	while(BusyXLCD());      // Wait for LCD to finish processing  
	putrsXLCD("                                ");	
	while(BusyXLCD());      // Wait for LCD to finish processing    */
      BUZ1 = 0x00;   // turn ON the LED 
}
////////////////////////////////////////////////////////////////////////
void keyboard(void)
{
    
	// Bits 4 -7 of Port D are configured as Outputs connected to Column 1 - 3 of the matrix keyboard
	/* Set Port D as Digital Outputs to show the response from the switches to LEDs */
        ADCON1 = 0x0E; // All ports I/Ps set to digital
	INTCON2bits.RBPU = 0; // Set Internal port B Pull Ups
	TRISB = 0x1E; // Configure Most Significant Nibble of Port B as O/Ps,
	LATB = 0x00; // Initialise Port 

         KEYBUFF[i] = 0x00;
	// and Least to inputs for Row scanning
         do
	// Read Port B Input switches and display on LEDs at Port D
	{
        //===================
	// **** Scan Columns make column 1 zero *****
		LATB = 0b11000000; // Column 1 firstly set to zero
		Delay1KTCYx(2); // Debounce delay
		if (PORTBbits.RB1==0) // Check Row 1 is it Key 1?
		KEYBUFF[i] = 1; // Display a key Value on O/P
		if (PORTBbits.RB2==0) // Check for next key in Row 2, is it key 4?
		KEYBUFF[i] = 4; // If so display key 4
		if (PORTBbits.RB3==0) // Check for next key in Row 3, is it key 7?
		KEYBUFF[i] = 7; // If so display key 7
		if (PORTBbits.RB4==0) // Check for next key in Row 4, is it key *?
		KEYBUFF[i] = 80 ;
	// *** Make column 2 zero and check for key ****
		LATB = 0b10100000; // Column 2 set to zero
		Delay1KTCYx(2); // Debounce delay
		if (PORTBbits.RB1==0) // Check Row 1 is it Key 2?
		KEYBUFF[i] = 2; // Display the key Value on O/P
		if (PORTBbits.RB2==0) // Check Row 2 is it Key 5?
		KEYBUFF[i] = 5; // Display the key Value on O/P
		if (PORTBbits.RB3==0) // Check Row 3 is it Key 8?
		KEYBUFF[i] = 8; // Display the key Value on O/P
		if (PORTBbits.RB4==0) // Check for next key in Row 4, is it key 0?
		KEYBUFF[i] = 0; // Display the key Value on O/P
		// *** Make Column 3 zero and check for key ***
		LATB = 0b01100000; // Column 3 set to zero
		Delay1KTCYx(2); // Debounce delay
		if (PORTBbits.RB1==0) // Check Row 1 is it Key 3?
		KEYBUFF[i] = 3; // if so display the key Value on O/P
		if (PORTBbits.RB2==0) // Check Row 2 is it Key 6?
		KEYBUFF[i] = 6; // if so display the key Value on O/P
		if (PORTBbits.RB3==0) // Check Row 3 is it Key 9?
		KEYBUFF[i] = 9; // if so display the key Value on O/P
		if (PORTBbits.RB4==0) // Check Row 4 is it Key #?
		KEYBUFF[i] = 23; // If so show 0x20 on PortD LEDs

		
	
	}
        while ((KEYBUFF[i]) == (0));

	 ASCIIkeyDigit[i] = KEYBUFF[i] + 0x30;
	 if (KEYBUFF[i] == 23)
	{
	 ASCIIkeyDigit[i] = 0x23;                  //for hashtag
	}
	 Delay1KTCYx(2);
 
}
//////////////////////////////////////////////////////////////////////

void VirtTermMode(void)
{

  while(!DataRdyUSART()) ;     // Wait for user input
  pin[0] = getcUSART();  	         // Get character from terminal
  putcUSART(pin[0]);                // Display character entered
   while(BusyUSART());
   
   while(!DataRdyUSART()) ;     // Wait for user input
    pin[1] = getcUSART();  	         // Get character from terminal
  putcUSART(pin[1]);                // Display character entered
    while(BusyUSART());
  
  while(!DataRdyUSART()) ;     // Wait for user input
    pin[2] = getcUSART();  	         // Get character from terminal
  putcUSART(pin[2]);                // Display character entered
    while(BusyUSART());
    
      while(!DataRdyUSART()) ;     // Wait for user input
    pin[3] = getcUSART();  	         // Get character from terminal
  putcUSART(pin[3]);                // Display character entered
    while(BusyUSART());
    
    	 ASCIIkeyDigit[7] = pin[4] + 0x30;
if (((pin[0]) == (ASCIIDigit0)) && ((pin[1]) == (ASCIIDigit1)) && ((pin[2]) == (ASCIIDigit2)) && ((pin[3]) == (35)))
 {
 putrsUSART("\r\nUNLOCKING\r\n");
 while(BusyUSART());

 unlocked();
}
else 
{ 
putrsUSART("\r\n ATTEMPT N");
 while(BusyUSART());
x = x + 1;
p = x + 0x30;

 putcUSART(p);   
  while(BusyUSART()); 
  
  putrsUSART("\r\n");
 while(BusyUSART());
 
//////////////////////////////////
if(x == 1)
{
 	D3 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
	}
if (x == 2)
	{

	D3 = 1;
	D4 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	D4 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
	}
	

	
//////////////////////////////////////////
 
 
  if (x == 3)
  {
   putrsUSART("\r\nKeyboardLocked\r\n");
   while(BusyUSART());
   
   ///////////////////////
	D3 = 1;
	D4 = 1;
	D5 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	D5 = 0;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 1;
	D4 = 1;
	D5 = 1;
	BUZ1 = 1;
	Delay1KTCYx(255);
	D3 = 0;
	D4 = 0;
	D5 = 0;
	BUZ1 = 0;
	Delay1KTCYx(255);
   //////////////////////

	   main(); 
   }

}

    

}


//////////////////////////////////////////////////////////////////////////////////FOR LCD
//================ CUSTOM DELAY FUNCTION 1 =================================*/
void DelayFor18TCY(void)            // Delay for 18 Instr. cycles using NOPs*/
{                                   //                                      */
_asm NOP _endasm                    //                                      */				    
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
_asm NOP _endasm                    //                                      */
}                                   //                                      */
//=================== CUSTOM DELAY FUNCTION 2 ==============================*/
void DelayPORXLCD(void)
{            // DelayPORXLCD = ~15 ms. LCD required  */
	Delay1KTCYx(75);                //                                      */
}                                   //                                      */
//=================== CUSTOM DELAY FUNCTION 2 ==============================*/
void DelayXLCD(void)
{               // DelayXLCD = ~5 ms.     LCD required  */
	Delay1KTCYx(25);                //                                      */
}

