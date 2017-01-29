/*
  CharliePlexM uses
  
   uSTimer2.h - Using timer2 with 24uS resolution 

  
  
  History:
	2014/03/20 - v0.8   by E Porter - repair encoder not work by itself
    2014/02/10 - v0.7   by E Porter - repair encoder code
	2014/02/10 - v0.6   by E Porter - repair temporal anomaly, encoders
    2014/01/31 - v0.5   by E Porter - add button support
    2014/01/27 - v0.4   by E Porter - Make pin variable and # of pin variable
    2014/01/08 - v0.3   by E Porter - updated uSTimer2 control
    2013/10/31 - v0.2   by E Porter for MSE 2202B taken for CharliePlexing
  	2012/01/25 - V0.1   by E Porter for MSE 2202B 
	

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
*/


/*
to use Charieplexing
include both

#include <uSTimer2.h>
#include <CharliePlexM.h>

for 3 wire 6 LEDs (LEDs number 1, 2, 3, 10, 11, 12) use
  CharliePlexM::set(Dx1, Dx2, Dx3);
for 4 wire 12 LEDs  use
  CharliePlexM::set(Dx1, Dx2, Dx3, DX4);
for 3 wire 6 LEDs (LEDs number 1, 2, 3, 10, 11, 12) And Push Button use
  CharliePlexM::set(Dx1, Dx2, Dx3, PB);
for 4 wire 12 LEDs  And Push Button use
  CharliePlexM::set(Dx1, Dx2, Dx3, DX4, PB);
for Encoder use
   CharliePlexM::setEncoders( En1, En2);
  
Dx = Arduino's Digital Port number,  to wire so LED1 is LED1 etc.  wire Dx1 to C1, DX2, to C2 ....
PB = Digital port Number that button is connected to must be same as one of Dx number already used 
En1 and En2 are the Arduino's Digital Port number eth encoders are connected to and should NOT be the same as Dx or PB 

To change LED(s) brightness
CharliePlexM::Brightness(number form 1 to 10); //1 being dimmest

To turn on or off single LED
 CharliePlexM::Write(LED number,[1 for on/0 for off]);
 
 
to turn on matrix of LEDs
 CharliePlexM::WriteMatrix(LED Matrix Integer);
The matrix integer 
                      1963 1852 1741
                      2    1    0
uiLEDMatrixMSB = 0000 0000  
    uiLEDMatrixLSB =       0000 0000 
 
example for F  -  LED Matrix Integer = 0x0F55


  Led Layout                           
		             set( vc1,     vc2,     vc3)
		    set( vc1,     vc2,     vc3,     vc4) vc = charlieplex port variable 
	    	         DX1,     Dx2,     Dx3,     Dx4  DX = arduino's Digital Ports
			 C1 ,     C2 ,     C3 ,     C4   Chaplieplex pins on MSE-duino brd
			  |        |        |        |
			  |        |      LED1>      |
			  |        | <LED2  |        |
			  |        | LED3>  |        |
			  |      <LED4               |
			  |      LED5>               |
			  |     <LED6       |        |
			  |     LED7>       |        |
			  |  <LED8 |        |        |
			  |  LED9> |        |        |
			  |        |        | <LED10 |
			  |        |    <LED11       |
			  |        |        | LED12> |






The matrix command using in the serial monitor to enter numbers
                      1963 1852 1741
                      2    1    0
uiLEDMatrixMSB = 0000 0000  
    uiLEDMatrixLSB =       0000 0000 
 ie to print a letter A = uiLEDMatrixMSB = 15, uiLEDMatrixLSB = 95

*/



#include <math.h>
#include "CharliePlexM.h"

volatile unsigned char CharliePlexM::sc_Plexing_Index;
volatile unsigned char  CharliePlexM::uc_ButtonImageInput;
volatile unsigned int  CharliePlexM::ui_LED_Output_Image;
volatile unsigned int CharliePlexM::ui_PortD_Input;
volatile unsigned int CharliePlexM::uiMultiUse_uSTimer2;
volatile unsigned int  CharliePlexM::ui_Brightness;
volatile unsigned int  CharliePlexM::ui_Brightness_Working;
volatile unsigned int  CharliePlexM::ui_BtnPressCount;
volatile unsigned int  CharliePlexM::ui_BtnReleasedCount;
volatile unsigned int  CharliePlexM::ui_Btn;
volatile unsigned int  CharliePlexM::ui_BtnDebounceTime;
volatile unsigned int  CharliePlexM::ui_BtnIndexCount;
volatile unsigned int  CharliePlexM::ui_BtnPortNumber;

volatile unsigned int  CharliePlexM::uiSetIt_IsSet;



volatile unsigned int  CharliePlexM::ui_Encoder1_Port;
volatile unsigned int  CharliePlexM::ui_Encoder2_Port;
volatile unsigned int  CharliePlexM::ui_Encoder1_PortNumber;
volatile unsigned int  CharliePlexM::ui_Encoder2_PortNumber;
volatile unsigned int  CharliePlexM::ui_Encoder3_Port;
volatile unsigned int  CharliePlexM::ui_Encoder4_Port;
volatile unsigned int  CharliePlexM::ui_Encoder3_PortNumber;
volatile unsigned int  CharliePlexM::ui_Encoder4_PortNumber;

volatile unsigned long CharliePlexM::ul_LeftEncoder_Count;
volatile unsigned long CharliePlexM::ul_RightEncoder_Count;


volatile unsigned long CharliePlexM::ul_Encoder3_Count;
volatile unsigned long CharliePlexM::ul_Encoder4_Count;

//volatile unsigned int  CharliePlexM::uc_test[100];



void (*CharliePlexM::MSE_DualPortedRam_Func)();

static  char sc_Button_Support = 0;
static  char sc_ButtonOnWhichPort = 0; // 0 = port d, 1 port B
static char sc_PortDDirectionClearMask = 0xFF;
static char sc_PortBDirectionClearMask = 0xFF;
static char sc_Plexing_IndexLimit = 13;

static char sc_LeftEncoderImage = 0;
static char sc_RightEncoderImage = 0;
static char sc_Last_LeftEncoderImage = 0;
static char sc_Last_RightEncoderImage = 0;

static char sc_LeftEncoderImage2 = 0;
static char sc_RightEncoderImage2 = 0;
static char sc_Last_LeftEncoderImage2 = 0;
static char sc_Last_RightEncoderImage2 = 0;


static char sc_PortBDirectionSetMask[13];
static char sc_PortDDirectionSetMask[13];

static char sc_PortDLEDMask[13];
static char sc_PortBLEDMask[13];


static  int si_LEDImageIndex[12];


/*static const char scc_PortDirectionClearMask = 0x0F;	//B0000 1111;   //clear /inputs
static const char scc_PortDirectionSetMask[12] = { 0xAF,	//B1010 1111,   //led1
													 0x6F,	//B0110 1111,   //led2
													 0x6F,	//B0110 1111,   //led3
													 0x9F,	//B1001 1111,   //led4
													 0x9F,	//B1001 1111,   //led5	
													 0x5F,	//B0101 1111,   //led6	
													 0x5F,	//B0101 1111,    //led7
													 0x3F,	//B0011 1111,   //led8
													 0x3F,	//B0011 1111,   //led9
													 0xCF,	//B1100 1111,   //led10
													 0xAF,	//B1010 1111,   //led11
													 0xCF};	//B1100 1111,   //led12
													 
   												 																		
																								
	static const char scc_PortLEDMask[12] = {0x020,		//B0010 0000,  //led1 on
											0x040,		//B0100 0000,   //led2 on
											0x020,		//B0010 0000,   //led3 on
											0x080,		//B1000 0000,   //led4 on 
											0x010,		//B0001 0000,   //led5 on
											0x040,		//B0100 0000,   //led6 on
											0x010,		//B0001 0000,  //led7 on
											0x020,		//B0010 0000,   //led8 on
											0x010,		//B0001 0000,   //led9 on
											0x080,		//B1000 0000,   //led10 on 
											0x080,		//B1000 0000,   //led11 on
											0x040};		//B0100 0000,   //led12 on
																	 
																   
	static const int sci_LEDImageIndex[12] = {0x0001,		//B00000000 00000001,  //led1 check
											 0x0002,		//B00000000 00000010,   //led2 check
											 0x0004,		//B00000000 00000100,   //led3 check
											 0x0008,		//B00000000 00001000,   //led 4 check 
					   						 0x0010,		//B00000000 00010000,   //led5 check
											 0x0020,		//B00000000 00100000,  //led6 check
											 0x0040,		//B00000000 01000000,   //led7 check
											 0x0080,		//B00000000 10000000,   //led8 check
											 0x0100,		//B00000001 00000000,   //led 9 check 
					   						 0x0200, 		//B00000010 00000000,   //led10 check
											 0x0400,		//B00000100 00000000,   //led 11 check 
					   						 0x0800};		//B00001000 00000000,   //led12 check
																																		   

	*/
	
	

void CharliePlexM::CharliePlex() 
{


 #ifdef TimingTestFull
    
	DDRD |= 0x004;    // d3
	PORTD |= 0x004;
 
 #endif

	if(uiMultiUse_uSTimer2)
	{
		(*MSE_DualPortedRam_Func)();
	}
#ifdef TimingTest
	DDRD |= 0x004;   //port d3
	PORTD |= 0x004;
 
#endif

/*	PORTD maps to Arduino digital pins 0 to 7
    7654 3210
	DDDD DDDD
	7654 3210  
		DDRD - The Port D Data Direction Register - read/write 1 = output 0 = input
		PORTD - The Port D Data Register - read/write
		PIND - The Port D Input Pins Register - read only 

	PORTB maps to Arduino digital pins 8 to 13 The two high bits (6 & 7) map to the crystal pins and are not usable
    7654 3210
	ooDD DDDD
	ss11 1198
	cc32 10
		DDRB - The Port B Data Direction Register - read/write
		PORTB - The Port B Data Register - read/write
		PINB - The Port B Input Pins Register - read only 

	PORTC maps to Arduino analog pins 0 to 5. Pins 6 & 7 are only accessible on the Arduino Mini
    7654 3210
	 rAA AAAA
	 s54 3210
	 t
		DDRC - The Port C Data Direction Register - read/write
		PORTC - The Port C Data Register - read/write
		PINC - The Port C Input Pins Register - read only 
		
	             Set( vc1,     vc2)	
		         set( vc1,     vc2,     vc3)
		set( vc1,     vc2,     vc3,     vc4) vc = charlieplex port variable 
		     DX1,     Dx2,     Dx3,     Dx4  DX = arduino's Digital Ports
			 C1 ,     C2 ,     C3 ,     C4   Chaplieplex pins on MSE-duino brd
			  |        |        |        |
			  |        |      LED1>      |
			  |        | <LED2  |        |
			  |        | LED3>  |        |
			  |      <LED4               |
			  |      LED5>               |
			  |     <LED6       |        |
			  |     LED7>       |        |
			  |  <LED8 |        |        |
			  |  LED9> |        |        |
			  |        |        | <LED10 |
			  |        |      <LED11     |
			  |        |        | LED12> |
			  
			  
			  
   */                                                                           
	
	
	
	
	if(sc_PortDDirectionClearMask != 0x0FF)
	{
		DDRD &= sc_PortDDirectionClearMask;		//set directions to inputs
		PORTD &= sc_PortDDirectionClearMask;	//set portD 4,5,6,7 to 0
	}
	if(sc_PortBDirectionClearMask != 0x0FF)
	{
		DDRB &= sc_PortBDirectionClearMask;		//set directions to inputs
		PORTB &= sc_PortBDirectionClearMask;	//set portD 4,5,6,7 to 0
	}
	
	
	
	if(ui_Brightness_Working <= 0)
	{
		ui_Brightness_Working =  ui_Brightness;
		
		if((ui_LED_Output_Image & si_LEDImageIndex[sc_Plexing_Index]) && (sc_Plexing_Index <= (sc_Plexing_IndexLimit - 1 - sc_Button_Support)))
		{
			if(sc_PortDDirectionClearMask != 0x0FF)
			{
				PORTD |= sc_PortDLEDMask[sc_Plexing_Index];
				DDRD |= sc_PortDDirectionSetMask[sc_Plexing_Index];  //set directions to chosen led
			}
			if(sc_PortBDirectionClearMask != 0x0FF)
			{
				PORTB |= sc_PortBLEDMask[sc_Plexing_Index];
				DDRB |= sc_PortBDirectionSetMask[sc_Plexing_Index];  //set directions to chosen led
			}
			
			
			
		}
	    if(sc_Button_Support)
		{
			ui_BtnDebounceTime = 80;///uSTimer2::uSecs);
			
			if(sc_Plexing_Index == (sc_Plexing_IndexLimit - 1))
			{
				
				if(sc_PortDDirectionClearMask != 0x0FF)
				{
					PORTD |= sc_PortDLEDMask[sc_Plexing_Index];
					DDRD |= sc_PortDDirectionSetMask[sc_Plexing_Index];  //set directions to chosen led
				}
				if(sc_PortBDirectionClearMask != 0x0FF)
				{
					PORTB |= sc_PortBLEDMask[sc_Plexing_Index];
					DDRB |= sc_PortBDirectionSetMask[sc_Plexing_Index];  //set directions to chosen led
				}
				
				if(sc_ButtonOnWhichPort)
				{
					uc_ButtonImageInput = (PINB & ui_BtnPortNumber);
				}
				else
				{
					uc_ButtonImageInput = (PIND & ui_BtnPortNumber);
				}
				
				
				if(ui_Btn)
				{
					if(uc_ButtonImageInput == 0)
					{
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = 0;
					}
					else
					{
						// button is released
						ui_BtnPressCount = 0;
						ui_BtnReleasedCount = ui_BtnReleasedCount + 1;
						if(ui_BtnReleasedCount >= 25)//ui_BtnDebounceTime)
						{
						   ui_Btn = 0;
						   ui_BtnReleasedCount = 0;
						}
											
					}
				}
				else	// if button was not pressed
				{
					if(uc_ButtonImageInput == 0)
					{
						// button is pressed
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = ui_BtnPressCount + 1;
						
						if(ui_BtnPressCount >= 25)//ui_BtnDebounceTime)
						{
						   ui_Btn = 1;
						   ui_BtnPressCount = 0;
						}
					}
					else
					{
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = 0;
					}
				}
				
				
				
			}
		}
		
		
		sc_Plexing_Index++;
		if(sc_Plexing_Index > sc_Plexing_IndexLimit)
		{
			sc_Plexing_Index = 0;
		}
	}
	else
	{
	
		if(sc_Button_Support)
		{
		  ui_BtnDebounceTime = 80;///uSTimer2::uSecs);
		  if(ui_BtnIndexCount == (sc_Plexing_IndexLimit - 1))
			{
				if(sc_PortDDirectionClearMask != 0x0FF)
				{
					PORTD |= sc_PortDLEDMask[ui_BtnIndexCount];
					DDRD |= sc_PortDDirectionSetMask[ui_BtnIndexCount];  //set directions to chosen led
				}
				if(sc_PortBDirectionClearMask != 0x0FF)
				{
					PORTB |= sc_PortBLEDMask[ui_BtnIndexCount];
					DDRB |= sc_PortBDirectionSetMask[ui_BtnIndexCount];  //set directions to chosen led
				}
				if(sc_ButtonOnWhichPort)
				{
					uc_ButtonImageInput = (PINB & ui_BtnPortNumber);
				}
				else
				{
					uc_ButtonImageInput = (PIND & ui_BtnPortNumber);
				}
				if(ui_Btn)
				{
					if(uc_ButtonImageInput == 0)
					{
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = 0;
					}
					else
					{
						// button is released
						ui_BtnPressCount = 0;
						ui_BtnReleasedCount = ui_BtnReleasedCount + 1;
						if(ui_BtnReleasedCount >= 25)//ui_BtnDebounceTime)
						{
						   ui_Btn = 0;
						   ui_BtnReleasedCount = 0;
						}
											
					}
				}
				else	// if button was not pressed
				{
					if(uc_ButtonImageInput == 0)
					{
						// button is pressed
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = ui_BtnPressCount + 1;
						if(ui_BtnPressCount >= 25)//ui_BtnDebounceTime)
						{
						   ui_Btn = 1;
						   ui_BtnPressCount = 0;
						}
					}
					else
					{
						ui_BtnReleasedCount = 0;
						ui_BtnPressCount = 0;
					}
				}
			}
		
			ui_BtnIndexCount++;
			if(ui_BtnIndexCount > sc_Plexing_IndexLimit)
			{
				ui_BtnIndexCount = 0;
			} 
		} 
		
	
	}
	
	ui_Brightness_Working = ui_Brightness_Working - 1;
	
	//Encoders
	
	if(ui_Encoder1_Port == 1 )
	{
	 
	  sc_LeftEncoderImage = (PIND & ui_Encoder1_PortNumber);
		if(sc_LeftEncoderImage ^ sc_Last_LeftEncoderImage)
		{
			sc_Last_LeftEncoderImage = sc_LeftEncoderImage;
			ul_LeftEncoder_Count++;
		}
	}
	if(ui_Encoder1_Port == 2 )
	{
	  sc_LeftEncoderImage = (PINB & ui_Encoder1_PortNumber);
		if(sc_LeftEncoderImage ^ sc_Last_LeftEncoderImage)
		{
			sc_Last_LeftEncoderImage = sc_LeftEncoderImage;
			ul_LeftEncoder_Count++;
		}
	}
	if(ui_Encoder2_Port == 1 )
	{
	
		sc_RightEncoderImage  = (PIND & ui_Encoder2_PortNumber);
		if(sc_RightEncoderImage ^ sc_Last_RightEncoderImage)
		{
			sc_Last_RightEncoderImage = sc_RightEncoderImage;
			ul_RightEncoder_Count++;
		}
	}	
	if(ui_Encoder2_Port == 2 )
	{
		sc_RightEncoderImage  = (PINB & ui_Encoder2_PortNumber);
		if(sc_RightEncoderImage ^ sc_Last_RightEncoderImage)
		{
			sc_Last_RightEncoderImage = sc_RightEncoderImage;
			ul_RightEncoder_Count++;
		}
	}	
	
	if(ui_Encoder3_Port == 1 )
	{
	 
	  sc_LeftEncoderImage2 = (PIND & ui_Encoder3_PortNumber);
		if(sc_LeftEncoderImage2 ^ sc_Last_LeftEncoderImage2)
		{
			sc_Last_LeftEncoderImage2 = sc_LeftEncoderImage2;
			ul_Encoder3_Count++;
		}
	}
	if(ui_Encoder3_Port == 2 )
	{
	  sc_LeftEncoderImage2 = (PINB & ui_Encoder3_PortNumber);
		if(sc_LeftEncoderImage2 ^ sc_Last_LeftEncoderImage2)
		{
			sc_Last_LeftEncoderImage2 = sc_LeftEncoderImage2;
			ul_Encoder3_Count++;
		}
	}
	if(ui_Encoder4_Port == 1 )
	{
	
		sc_RightEncoderImage2  = (PIND & ui_Encoder4_PortNumber);
		if(sc_RightEncoderImage2 ^ sc_Last_RightEncoderImage2)
		{
			sc_Last_RightEncoderImage2 = sc_RightEncoderImage2;
			ul_Encoder4_Count++;
		}
	}	
	if(ui_Encoder4_Port == 2 )
	{
		sc_RightEncoderImage2  = (PINB & ui_Encoder4_PortNumber);
		if(sc_RightEncoderImage2 ^ sc_Last_RightEncoderImage2)
		{
			sc_Last_RightEncoderImage2 = sc_RightEncoderImage2;
			ul_Encoder4_Count++;
		}
	}	
	
		
		
		
#ifdef TimingTestFull
	
	PORTD &= 0xfb;
	
#endif	
#ifdef TimingTest
	
	PORTD &= 0xfb;
	
#endif	
}

void CharliePlexM::setEncoders(char b_Encoder1,char b_Encoder2,char b_Encoder3,char b_Encoder4)
{
	unsigned int ui_Inverter = 0xFFFF;
	if(b_Encoder1 < 8)  //port D
	{
	    ui_Encoder1_Port = 1;
		ui_Encoder1_PortNumber = round(pow(2,b_Encoder1));
		DDRD &= char(ui_Inverter ^ ui_Encoder1_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder1_PortNumber;
		
	}
	else  //portB
	{
		ui_Encoder1_Port = 2;
	    ui_Encoder1_PortNumber = round(pow(2,(b_Encoder1 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder1_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder1_PortNumber;
		
	}
	if(b_Encoder2 < 8)  //port D
	{
	    ui_Encoder2_Port = 1;
		ui_Encoder2_PortNumber = round(pow(2,b_Encoder2));
		DDRD &= char(ui_Inverter ^ ui_Encoder2_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder2_PortNumber;
		
		
	}
	else  //portB
	{
		ui_Encoder2_Port = 2;
	    ui_Encoder2_PortNumber = round(pow(2,(b_Encoder2 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder2_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder2_PortNumber;
	}
	
	if(b_Encoder3 < 8)  //port D
	{
	    ui_Encoder3_Port = 1;
		ui_Encoder3_PortNumber = round(pow(2,b_Encoder3));
		DDRD &= char(ui_Inverter ^ ui_Encoder3_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder3_PortNumber;
		
	}
	else  //portB
	{
		ui_Encoder3_Port = 2;
	    ui_Encoder3_PortNumber = round(pow(2,(b_Encoder3 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder3_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder3_PortNumber;
		
	}
	if(b_Encoder4 < 8)  //port D
	{
	    ui_Encoder4_Port = 1;
		ui_Encoder4_PortNumber = round(pow(2,b_Encoder4));
		DDRD &= char(ui_Inverter ^ ui_Encoder4_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder4_PortNumber;
		
		
	}
	else  //portB
	{
		ui_Encoder4_Port = 2;
	    ui_Encoder4_PortNumber = round(pow(2,(b_Encoder4 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder4_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder4_PortNumber;
	}
    setIt();
}

void CharliePlexM::setEncoders(char b_Encoder1,char b_Encoder2)
{
	unsigned int ui_Inverter = 0xFFFF;
	if(b_Encoder1 < 8)  //port D
	{
	    ui_Encoder1_Port = 1;
		ui_Encoder1_PortNumber = round(pow(2,b_Encoder1));
		DDRD &= char(ui_Inverter ^ ui_Encoder1_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder1_PortNumber;
		
	}
	else  //portB
	{
		ui_Encoder1_Port = 2;
	    ui_Encoder1_PortNumber = round(pow(2,(b_Encoder1 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder1_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder1_PortNumber;
		
	}
	if(b_Encoder2 < 8)  //port D
	{
	    ui_Encoder2_Port = 1;
		ui_Encoder2_PortNumber = round(pow(2,b_Encoder2));
		DDRD &= char(ui_Inverter ^ ui_Encoder2_PortNumber);		//set directions to inputs
		PORTD &= ui_Encoder2_PortNumber;
		
		
	}
	else  //portB
	{
		ui_Encoder2_Port = 2;
	    ui_Encoder2_PortNumber = round(pow(2,(b_Encoder2 - 8)));
		DDRB &= char(ui_Inverter ^ ui_Encoder2_PortNumber);		//set directions to inputs
		PORTB &= ui_Encoder2_PortNumber;
	}
    setIt();
}

void CharliePlexM::set(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_ChaliePort4) 
{

	unsigned int ui_CPort1;
	unsigned int ui_CPort2;
	unsigned int ui_CPort3;
	unsigned int ui_CPort4;
	
	
	
	sc_Plexing_IndexLimit = 12 + sc_Button_Support;
	sc_PortDDirectionClearMask = 0x0FF;
	sc_PortBDirectionClearMask = 0x0FF;
	
	sc_PortDDirectionSetMask[12] = 0;
	sc_PortDLEDMask[12] = 0;
	sc_PortBDirectionSetMask[12] = 0;
	sc_PortBLEDMask[12] = 0;
	
	si_LEDImageIndex[0] = 0x0001;		//B00000000 00000001,  //led1 check
	si_LEDImageIndex[1] = 0x0002;		//B00000000 00000010,   //led2 check
	si_LEDImageIndex[2] = 0x0004;		//B00000000 00000100,   //led3 check
	si_LEDImageIndex[3] = 0x0008;		//B00000000 00001000,   //led 4 check 
	si_LEDImageIndex[4] = 0x0010;		//B00000000 00010000,   //led5 check
	si_LEDImageIndex[5] = 0x0020;		//B00000000 00100000,  //led6 check
	si_LEDImageIndex[6] = 0x0040;		//B00000000 01000000,   //led7 check
	si_LEDImageIndex[7] = 0x0080;		//B00000000 10000000,   //led8 check
	si_LEDImageIndex[8] =  0x0100;		//B00000001 00000000,   //led 9 check 
	si_LEDImageIndex[9] = 0x0200; 		//B00000010 00000000,   //led10 check
	si_LEDImageIndex[10] = 0x0400;		//B00000100 00000000,   //led 11 check 
	si_LEDImageIndex[11] = 0x0800;		//B00001000 00000000,   //led12 check
	
	
    if(b_ChaliePort1 < 8)
	{
	    ui_CPort1 = round(pow(2,b_ChaliePort1));
		sc_PortDDirectionClearMask ^= ui_CPort1;
		sc_PortDDirectionSetMask[12] |= ui_CPort1;
		sc_PortDLEDMask[12] |= ui_CPort1;
		
	}
	else
	{
	    ui_CPort1 = round(pow(2,(b_ChaliePort1 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort1;
		sc_PortBDirectionSetMask[12] |= ui_CPort1;
		sc_PortBLEDMask[12] |= ui_CPort1;
	}
	if(b_ChaliePort2 < 8)
	{
	    ui_CPort2 = round(pow(2,b_ChaliePort2));
		sc_PortDDirectionClearMask ^= ui_CPort2;
		sc_PortDDirectionSetMask[12] |= ui_CPort2;
		sc_PortDLEDMask[12] |= ui_CPort2;
	}
	else
	{
	    ui_CPort2 = round(pow(2,(b_ChaliePort2 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort2;
		sc_PortBDirectionSetMask[12] |= ui_CPort2;
		sc_PortBLEDMask[12] |= ui_CPort2;
	}
	if(b_ChaliePort3 < 8)
	{
	    ui_CPort3 = round(pow(2,b_ChaliePort3));
		sc_PortDDirectionClearMask ^= ui_CPort3;
		sc_PortDDirectionSetMask[12] |= ui_CPort3;
		sc_PortDLEDMask[12] |= ui_CPort3;
		
	}
	else
	{
	    ui_CPort3 = round(pow(2,(b_ChaliePort3 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort3;
		sc_PortBDirectionSetMask[12] |= ui_CPort3;
		sc_PortBLEDMask[12] |= ui_CPort3;
	}
	if(b_ChaliePort4 < 8)
	{
	    ui_CPort4 = round(pow(2,b_ChaliePort4));
		sc_PortDDirectionClearMask ^= ui_CPort4;
		sc_PortDDirectionSetMask[12] |= ui_CPort4;
		sc_PortDLEDMask[12] |= ui_CPort4;
	}
	else
	{
	    ui_CPort4 = round(pow(2,(b_ChaliePort4 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort4;
		sc_PortBDirectionSetMask[12] |= ui_CPort4;
		sc_PortBLEDMask[12] |= ui_CPort4;
	}
	
	
	//LED1 and LED 11
	sc_PortBDirectionSetMask[0] =  0;
	sc_PortDDirectionSetMask[0] =  0; 
	sc_PortBLEDMask[0] = 0;
	sc_PortDLEDMask[0] = 0;
	sc_PortBDirectionSetMask[10] =  0;
	sc_PortDDirectionSetMask[10] =  0; 
	sc_PortBLEDMask[10] = 0;
	sc_PortDLEDMask[10] = 0;
	if(b_ChaliePort2 < 8)
	{
		sc_PortDDirectionSetMask[0] |= ui_CPort2;
		sc_PortDDirectionSetMask[10] |= ui_CPort2;
		sc_PortDLEDMask[0] = ui_CPort2;
	}
	else
	{
		sc_PortBDirectionSetMask[0] |= ui_CPort2;
		sc_PortBDirectionSetMask[10] |= ui_CPort2;
		sc_PortBLEDMask[0] = ui_CPort2;
	}
	if(b_ChaliePort4 < 8)
	{
		sc_PortDDirectionSetMask[0] |= ui_CPort4;
		sc_PortDDirectionSetMask[10] |= ui_CPort4;
		sc_PortDLEDMask[10] = ui_CPort4;
		
	}
	else
	{
		sc_PortBDirectionSetMask[0] |= ui_CPort4;
		sc_PortBDirectionSetMask[10] |= ui_CPort4;
		sc_PortBLEDMask[10] = ui_CPort4;
	}
	//LED2 and LED 3
	sc_PortBDirectionSetMask[1] =  0;
	sc_PortDDirectionSetMask[1] =  0; 
	sc_PortBLEDMask[1] = 0;
	sc_PortDLEDMask[1] = 0;
	sc_PortBDirectionSetMask[2] =  0;
	sc_PortDDirectionSetMask[2] =  0; 
	sc_PortBLEDMask[2] = 0;
	sc_PortDLEDMask[2] = 0;
	if(b_ChaliePort2 < 8)
	{
		sc_PortDDirectionSetMask[1] |= ui_CPort2;
		sc_PortDDirectionSetMask[2] |= ui_CPort2;
		sc_PortDLEDMask[2] = ui_CPort2;
	}
	else
	{
		sc_PortBDirectionSetMask[1] |= ui_CPort2;
		sc_PortBDirectionSetMask[2] |= ui_CPort2;
		sc_PortBLEDMask[2] = ui_CPort2;
	}
	if(b_ChaliePort3 < 8)
	{
		sc_PortDDirectionSetMask[1] |= ui_CPort3;
		sc_PortDDirectionSetMask[2] |= ui_CPort3;
		sc_PortDLEDMask[1] = ui_CPort3;
	}
	else
	{
		sc_PortBDirectionSetMask[1] |= ui_CPort3;
		sc_PortBDirectionSetMask[2] |= ui_CPort3;
		sc_PortBLEDMask[1] = ui_CPort3;
	}
	//LED4 and LED 5
	sc_PortBDirectionSetMask[3] |=  0;
	sc_PortDDirectionSetMask[3] |=  0; 
	sc_PortBLEDMask[3] = 0;
	sc_PortDLEDMask[3] = 0;
	sc_PortBDirectionSetMask[4] |=  0;
	sc_PortDDirectionSetMask[4] |=  0; 
	sc_PortBLEDMask[4] = 0;
	sc_PortDLEDMask[4] = 0;
	if(b_ChaliePort1 < 8)
	{
		sc_PortDDirectionSetMask[3] |= ui_CPort1;
		sc_PortDDirectionSetMask[4] |= ui_CPort1;
		sc_PortDLEDMask[4] = ui_CPort1;
	}
	else
	{
		sc_PortBDirectionSetMask[3] |= ui_CPort1;
		sc_PortBDirectionSetMask[4] |= ui_CPort1;
		sc_PortBLEDMask[4] = ui_CPort1;
	}
	if(b_ChaliePort4 < 8)
	{
		sc_PortDDirectionSetMask[3] |= ui_CPort4;
		sc_PortDDirectionSetMask[4] |= ui_CPort4;
		sc_PortDLEDMask[3] = ui_CPort4;
	}
	else
	{
		sc_PortBDirectionSetMask[3] |= ui_CPort4;
		sc_PortBDirectionSetMask[4] |= ui_CPort4;
		sc_PortBLEDMask[3] = ui_CPort4;
	}
	//LED6 and LED 7
	sc_PortBDirectionSetMask[5] =  0;
	sc_PortDDirectionSetMask[5] =  0; 
	sc_PortBLEDMask[5] = 0;
	sc_PortDLEDMask[5] = 0;
	sc_PortBDirectionSetMask[6] =  0;
	sc_PortDDirectionSetMask[6] =  0; 
	sc_PortBLEDMask[6] = 0;
	sc_PortDLEDMask[6] = 0;
	if(b_ChaliePort1 < 8)
	{
		sc_PortDDirectionSetMask[5] |= ui_CPort1;
		sc_PortDDirectionSetMask[6] |= ui_CPort1;
		sc_PortDLEDMask[6] = ui_CPort1;
	}
	else
	{
		sc_PortBDirectionSetMask[5] |= ui_CPort1;
		sc_PortBDirectionSetMask[6] |= ui_CPort1;
		sc_PortBLEDMask[6] = ui_CPort1;
	}
	if(b_ChaliePort3 < 8)
	{
		sc_PortDDirectionSetMask[5] |= ui_CPort3;
		sc_PortDDirectionSetMask[6] |= ui_CPort3;
		sc_PortDLEDMask[5] = ui_CPort3;
	}
	else
	{
		sc_PortBDirectionSetMask[5] |= ui_CPort3;
		sc_PortBDirectionSetMask[6] |= ui_CPort3;
		sc_PortBLEDMask[5] = ui_CPort3;
	}
	
	//LED8 and LED 9
	sc_PortBDirectionSetMask[7] =  0;
	sc_PortDDirectionSetMask[7] =  0; 
	sc_PortBLEDMask[7] = 0;
	sc_PortDLEDMask[7] = 0;
	sc_PortBDirectionSetMask[8] =  0;
	sc_PortDDirectionSetMask[8] =  0; 
	sc_PortBLEDMask[8] = 0;
	sc_PortDLEDMask[8] = 0;
	if(b_ChaliePort1 < 8)
	{
		sc_PortDDirectionSetMask[7] |= ui_CPort1;
		sc_PortDDirectionSetMask[8] |= ui_CPort1;
		sc_PortDLEDMask[8] = ui_CPort1;
	}
	else
	{
		sc_PortBDirectionSetMask[7] |= ui_CPort1;
		sc_PortBDirectionSetMask[8] |= ui_CPort1;
		sc_PortBLEDMask[8] = ui_CPort1;
	}
	if(b_ChaliePort2 < 8)
	{
		sc_PortDDirectionSetMask[7] |= ui_CPort2;
		sc_PortDDirectionSetMask[8] |= ui_CPort2;
		sc_PortDLEDMask[7] = ui_CPort2;
	}
	else
	{
		sc_PortBDirectionSetMask[7] |= ui_CPort2;
		sc_PortBDirectionSetMask[8] |= ui_CPort2;
		sc_PortBLEDMask[7] = ui_CPort2;
	}
	//LED10 and LED 12
	sc_PortBDirectionSetMask[9] =  0;
	sc_PortDDirectionSetMask[9] =  0; 
	sc_PortBLEDMask[9] = 0;
	sc_PortDLEDMask[9] = 0;
	sc_PortBDirectionSetMask[11] =  0;
	sc_PortDDirectionSetMask[11] =  0; 
	sc_PortBLEDMask[11] = 0;
	sc_PortDLEDMask[11] = 0;
	if(b_ChaliePort3 < 8)
	{
		sc_PortDDirectionSetMask[9] |= ui_CPort3;
		sc_PortDDirectionSetMask[11] |= ui_CPort3;
		sc_PortDLEDMask[11] = ui_CPort3;
	}
	else
	{
		sc_PortBDirectionSetMask[9] |= ui_CPort3;
		sc_PortBDirectionSetMask[11] |= ui_CPort3;
		sc_PortBLEDMask[11] = ui_CPort3;
	}
	if(b_ChaliePort4 < 8)
	{
		sc_PortDDirectionSetMask[9] |= ui_CPort4;
		sc_PortDDirectionSetMask[11] |= ui_CPort4;
		sc_PortDLEDMask[9] = ui_CPort4;
	}
	else
	{
		sc_PortBDirectionSetMask[9] |= ui_CPort4;
		sc_PortBDirectionSetMask[11] |= ui_CPort4;
		sc_PortBLEDMask[9] = ui_CPort4;
	}
	if(sc_Button_Support)
	{
		
		if(ui_BtnPortNumber < 8)
		{
			sc_ButtonOnWhichPort = 0;
			ui_BtnPortNumber = round(pow(2,ui_BtnPortNumber));
			sc_PortDDirectionSetMask[12] ^= ui_BtnPortNumber;
			
		}
		else
		{
		    sc_ButtonOnWhichPort = 1;
			ui_BtnPortNumber = round(pow(2,(ui_BtnPortNumber - 8)));
			sc_PortBDirectionSetMask[12] ^= ui_BtnPortNumber;
			
			
	    }
		
	}
	
	
	setIt();
   
}


void CharliePlexM::setBtn(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_ChaliePort4,char b_BtnPortNumber)
{
	
	ui_BtnPortNumber = b_BtnPortNumber;
	ui_BtnIndexCount = 0;

	
	
    sc_Button_Support = 1;
	set(b_ChaliePort1, b_ChaliePort2, b_ChaliePort3, b_ChaliePort4);
}


void CharliePlexM::setBtn(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_BtnPortNumber)
{
	
	ui_BtnPortNumber = b_BtnPortNumber;
	ui_BtnIndexCount = 0;
	
	
	
    sc_Button_Support = 1;
	set(b_ChaliePort1, b_ChaliePort2, b_ChaliePort3);
}

void CharliePlexM::set(char b_ChaliePort2,char b_ChaliePort3,char b_ChaliePort4) 
{
   
	unsigned int ui_CPort2;
	unsigned int ui_CPort3;
	unsigned int ui_CPort4;
	
	
	ui_Btn = 0;
	
	sc_Plexing_IndexLimit = 6 + sc_Button_Support;;
	sc_PortDDirectionClearMask = 0x0FF;
	sc_PortBDirectionClearMask = 0x0FF;
	
	sc_PortDDirectionSetMask[6] = 0;
	sc_PortDLEDMask[6] = 0;
	sc_PortBDirectionSetMask[6] = 0;
	sc_PortBLEDMask[6] = 0;
	
	si_LEDImageIndex[0] = 0x0001;		//B00000000 00000001,  //led1 check
	si_LEDImageIndex[1] = 0x0002;		//B00000000 00000010,   //led2 check
	si_LEDImageIndex[2] = 0x0004;		//B00000000 00000100,   //led3 check
	si_LEDImageIndex[3] = 0x0200; 		//B00000010 00000000,   //led10 check
	si_LEDImageIndex[4] = 0x0400;		//B00000100 00000000,   //led 11 check 
	si_LEDImageIndex[5] = 0x0800;		//B00001000 00000000,   //led12 check
	
	
   
	if(b_ChaliePort2 < 8)
	{
	    ui_CPort2 = round(pow(2,b_ChaliePort2));
		sc_PortDDirectionClearMask ^= ui_CPort2;
		sc_PortDDirectionSetMask[6] |= ui_CPort2;
		sc_PortDLEDMask[6] |= ui_CPort2;
	}
	else
	{
	    ui_CPort2 = round(pow(2,(b_ChaliePort2 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort2;
		sc_PortBDirectionSetMask[6] |= ui_CPort2;
		sc_PortBLEDMask[6] |= ui_CPort2;
	}
	if(b_ChaliePort3 < 8)
	{
	    ui_CPort3 = round(pow(2,b_ChaliePort3));
		sc_PortDDirectionClearMask ^= ui_CPort3;
		sc_PortDDirectionSetMask[6] |= ui_CPort3;
		sc_PortDLEDMask[6] |= ui_CPort3;
	}
	else
	{
	    ui_CPort3 = round(pow(2,(b_ChaliePort3 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort3;
		sc_PortBDirectionSetMask[6] |= ui_CPort3;
		sc_PortBLEDMask[6] |= ui_CPort3;
	}
	if(b_ChaliePort4 < 8)
	{
	    ui_CPort4 = round(pow(2,b_ChaliePort4));
		sc_PortDDirectionClearMask ^= ui_CPort4;
		sc_PortDDirectionSetMask[6] |= ui_CPort4;
		sc_PortDLEDMask[6] |= ui_CPort4;
	}
	else
	{
	    ui_CPort4 = round(pow(2,(b_ChaliePort4 - 8)));
		sc_PortBDirectionClearMask ^= ui_CPort4;
		sc_PortBDirectionSetMask[6] |= ui_CPort4;
		sc_PortBLEDMask[6] |= ui_CPort4;
	}
	
	
	//LED1 and LED 11
	sc_PortBDirectionSetMask[0] =  0;
	sc_PortDDirectionSetMask[0] =  0; 
	sc_PortBLEDMask[0] = 0;
	sc_PortDLEDMask[0] = 0;
	sc_PortBDirectionSetMask[4] =  0;
	sc_PortDDirectionSetMask[4] =  0; 
	sc_PortBLEDMask[4] = 0;
	sc_PortDLEDMask[4] = 0;
	if(b_ChaliePort2 < 8)
	{
		sc_PortDDirectionSetMask[0] |= ui_CPort2;
		sc_PortDDirectionSetMask[4] |= ui_CPort2;
		sc_PortDLEDMask[0] = ui_CPort2;
	}
	else
	{
		sc_PortBDirectionSetMask[0] |= ui_CPort2;
		sc_PortBDirectionSetMask[4] |= ui_CPort2;
		sc_PortBLEDMask[0] = ui_CPort2;
	}
	if(b_ChaliePort4 < 8)
	{
		sc_PortDDirectionSetMask[0] |= ui_CPort4;
		sc_PortDDirectionSetMask[4] |= ui_CPort4;
		sc_PortDLEDMask[4] = ui_CPort4;
		
	}
	else
	{
		sc_PortBDirectionSetMask[0] |= ui_CPort4;
		sc_PortBDirectionSetMask[4] |= ui_CPort4;
		sc_PortBLEDMask[4] = ui_CPort4;
	}
	//LED2 and LED 3
	sc_PortBDirectionSetMask[1] =  0;
	sc_PortDDirectionSetMask[1] =  0; 
	sc_PortBLEDMask[1] = 0;
	sc_PortDLEDMask[1] = 0;
	sc_PortBDirectionSetMask[2] =  0;
	sc_PortDDirectionSetMask[2] =  0; 
	sc_PortBLEDMask[2] = 0;
	sc_PortDLEDMask[2] = 0;
	if(b_ChaliePort2 < 8)
	{
		sc_PortDDirectionSetMask[1] |= ui_CPort2;
		sc_PortDDirectionSetMask[2] |= ui_CPort2;
		sc_PortDLEDMask[2] = ui_CPort2;
	}
	else
	{
		sc_PortBDirectionSetMask[1] |= ui_CPort2;
		sc_PortBDirectionSetMask[2] |= ui_CPort2;
		sc_PortBLEDMask[2] = ui_CPort2;
	}
	if(b_ChaliePort3 < 8)
	{
		sc_PortDDirectionSetMask[1] |= ui_CPort3;
		sc_PortDDirectionSetMask[2] |= ui_CPort3;
		sc_PortDLEDMask[1] = ui_CPort3;
	}
	else
	{
		sc_PortBDirectionSetMask[1] |= ui_CPort3;
		sc_PortBDirectionSetMask[2] |= ui_CPort3;
		sc_PortBLEDMask[1] = ui_CPort3;
	}
	
	//LED10 and LED 12
	sc_PortBDirectionSetMask[3] =  0;
	sc_PortDDirectionSetMask[3] =  0; 
	sc_PortBLEDMask[3] = 0;
	sc_PortDLEDMask[3] = 0;
	sc_PortBDirectionSetMask[5] =  0;
	sc_PortDDirectionSetMask[5] =  0; 
	sc_PortBLEDMask[5] = 0;
	sc_PortDLEDMask[5] = 0;
	if(b_ChaliePort3 < 8)
	{
		sc_PortDDirectionSetMask[3] |= ui_CPort3;
		sc_PortDDirectionSetMask[5] |= ui_CPort3;
		sc_PortDLEDMask[5] = ui_CPort3;
	}
	else
	{
		sc_PortBDirectionSetMask[3] |= ui_CPort3;
		sc_PortBDirectionSetMask[5] |= ui_CPort3;
		sc_PortBLEDMask[5] = ui_CPort3;
	}
	if(b_ChaliePort4 < 8)
	{
		sc_PortDDirectionSetMask[3] |= ui_CPort4;
		sc_PortDDirectionSetMask[5] |= ui_CPort4;
		sc_PortDLEDMask[3] = ui_CPort4;
	}
	else
	{
		sc_PortBDirectionSetMask[3] |= ui_CPort4;
		sc_PortBDirectionSetMask[5] |= ui_CPort4;
		sc_PortBLEDMask[3] = ui_CPort4;
	}
	
	if(sc_Button_Support)
	{
		
		if(ui_BtnPortNumber < 8)
		{
			sc_ButtonOnWhichPort = 0;
			ui_BtnPortNumber = round(pow(2,ui_BtnPortNumber));
			sc_PortDDirectionSetMask[6] ^= ui_BtnPortNumber;
			
		}
		else
		{
		    sc_ButtonOnWhichPort = 1;
			ui_BtnPortNumber = round(pow(2,(ui_BtnPortNumber - 8)));
			sc_PortBDirectionSetMask[6] ^= ui_BtnPortNumber;
			
			
	    }
		
	}
	
	
	
	setIt();
   
}


void CharliePlexM::setIt()
{
   
   if(uiSetIt_IsSet == 0)
   {
	   uiSetIt_IsSet = 1;
	   
	   ui_Brightness = 6;
	   ui_LED_Output_Image = 0;
	   uiMultiUse_uSTimer2  = 0;
	   if(uSTimer2::uSecs != 0)
	   {
		 uiMultiUse_uSTimer2 = 1;
		 MSE_DualPortedRam_Func = uSTimer2::func;
		 uSTimer2::stop();
	   }	 
	   uSTimer2::set(2, CharliePlex); //
	   uSTimer2::start();
	}   
}

void CharliePlexM::Brightness(unsigned int ui_Display_Brightness)
{

	if((ui_Display_Brightness <=  10) && (ui_Display_Brightness >= 1))
	{
		ui_Brightness = 11- ui_Display_Brightness;
	}
	else
	{
		ui_Brightness = 6;
	}
	
}

void CharliePlexM::Write(unsigned char uc_LED_Number,unsigned char uc_On_Off)
{

	const int ui_High_Mask[12] = {0x0001,		//B00000000 00000001,  //led1 on
								   0x0002,		//B00000000 00000010,   //led2 on
								   0x0004,		//B00000000 00000100,   //led3 on
								   0x0008,		//B00000000 00001000,   //led 4 on 
								   0x0010,		//B00000000 00010000,   //led5 on
								   0x0020,		//B00000000 00100000,  //led6 on
								   0x0040,		//B00000000 01000000,   //led7 on
								   0x0080,		//B00000000 10000000,   //led8 on
								   0x0100,		//B00000001 00000000,   //led 9 on 
								   0x0200, 		//B00000010 00000000,   //led10 on
								   0x0400,		//B00000100 00000000,   //led 11 on 
								   0x0800};		//B00001000 00000000,   //led12 on
								   
	const int ui_Low_Mask[12] = {0xFFFE,		//B11111111 1111 1110,  //led1 OFF
								  0xFFFD,		//B11111111 1111 1101,  //led2 OFF
								  0xFFFB,		//B11111111 1111 1011,  //led3 OFF
								  0xFFF7,		//B11111111 1111 0111,  //led4 OFF
								  0xFFEF,		//B11111111 1110 1111,  //led5 OFF
								  0xFFDF,		//B11111111 1101 1111,  //led6 OFF
								  0xFFBF,		//B11111111 1011 1111,  //led7 OFF
								  0xFF7F,		//B11111111 0111 1111,  //led8 OFF
								  0xFEFF,		//B11111110 1111 1111,  //led9 OFF
								  0xFDFF,		//B11111101 1111 1111,  //led10 OFF
								  0xFBFF,		//B11111011 1111 1111,  //led11 OFF
							      0xF7FF};		//B11110111 1111 1111,  //led12 OFF
	
	if((uc_LED_Number >= 1) && (uc_LED_Number <= 12))
	{
	    
		uc_LED_Number = uc_LED_Number - 1;
		if(uc_On_Off)
		{
			
			ui_LED_Output_Image |= ui_High_Mask[uc_LED_Number];
			
		}
		else
		{
			
			ui_LED_Output_Image &= ui_Low_Mask[uc_LED_Number];
			
			
		}
	}
	
	
}

void	CharliePlexM::WriteMatrix(unsigned int ui_LED_Matrix)
{
	//                     1963 1852 1741
	//                     2    1    0
	//ui_LED_Matrix = 0000 0000 0000 0000 	
	
	
	const int ui_High_Mask[12] = {0x0001,		//B00000000 00000001,  //led1 on
								   0x0002,		//B00000000 00000010,   //led2 on
								   0x0004,		//B00000000 00000100,   //led3 on
								   0x0008,		//B00000000 00001000,   //led 4 on 
								   0x0010,		//B00000000 00010000,   //led5 on
								   0x0020,		//B00000000 00100000,  //led6 on
								   0x0040,		//B00000000 01000000,   //led7 on
								   0x0080,		//B00000000 10000000,   //led8 on
								   0x0100,		//B00000001 00000000,   //led 9 on 
								   0x0200, 		//B00000010 00000000,   //led10 on
								   0x0400,		//B00000100 00000000,   //led 11 on 
								   0x0800};		//B00001000 00000000,   //led12 on
								   
	
	ui_LED_Output_Image = 0x0000;		
	
	if(ui_LED_Matrix & 0x0001)
	{
		ui_LED_Output_Image |= ui_High_Mask[0];
	}
	if(ui_LED_Matrix & 0x0010)
	{
		ui_LED_Output_Image |= ui_High_Mask[1];
	}
	if(ui_LED_Matrix & 0x0100)
	{
		ui_LED_Output_Image |= ui_High_Mask[2];
	}
	if(ui_LED_Matrix & 0x0002)
	{
		ui_LED_Output_Image |= ui_High_Mask[3];
	}
	if(ui_LED_Matrix & 0x0020)
	{
		ui_LED_Output_Image |= ui_High_Mask[4];
	}
	if(ui_LED_Matrix & 0x0200)
	{
		ui_LED_Output_Image |= ui_High_Mask[5];
	}
	if(ui_LED_Matrix & 0x0004)
	{
		ui_LED_Output_Image |= ui_High_Mask[6];
	}
	if(ui_LED_Matrix & 0x0040)
	{
		ui_LED_Output_Image |= ui_High_Mask[7];
	}
	if(ui_LED_Matrix & 0x0400)
	{
		ui_LED_Output_Image |= ui_High_Mask[8];
	}
	if(ui_LED_Matrix & 0x0008)
	{
		ui_LED_Output_Image |= ui_High_Mask[9];
	}
	if(ui_LED_Matrix & 0x0080)
	{
		ui_LED_Output_Image |= ui_High_Mask[10];
	}
	if(ui_LED_Matrix & 0x0800)
	{
		ui_LED_Output_Image |= ui_High_Mask[11];
	}
	
	
			
		
}

