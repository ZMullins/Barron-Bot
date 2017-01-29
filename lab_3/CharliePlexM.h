#ifndef CharliePlexM_h
#define CharliePlexM_h

#include "uSTimer2.h"

    //                     1963 1852 1741
	//                     2    1    0
	//ui_LED_Matrix = 0000 0000 0000 0000 	
#define M_LED1 0x0001 
#define M_LED2 0x0010
#define M_LED3 0x0100
#define M_LED4 0x0002
#define M_LED5 0x0020
#define M_LED6 0x0200
#define M_LED7 0x0004
#define M_LED8 0x0040
#define M_LED9 0x0400
#define M_LED10 0x0008
#define M_LED11 0x0080
#define M_LED12 0x0800

//#define TimingTestFull 1
//#define TimingTest 1

		
namespace CharliePlexM {
	
	extern volatile unsigned char sc_Plexing_Index;
	extern volatile unsigned char uc_ButtonImageInput;
	extern volatile unsigned int  ui_LED_Output_Image;
	extern volatile unsigned int  ui_PortD_Input;	
	extern volatile unsigned int  ui_Brightness;
	extern volatile unsigned int  ui_Brightness_Working;
	extern volatile unsigned int  ui_BtnPressCount;
	extern volatile unsigned int  ui_BtnReleasedCount;
	extern volatile unsigned int  ui_Btn;
	extern volatile unsigned int  ui_BtnDebounceTime;
	extern volatile unsigned int  ui_BtnIndexCount;
	extern volatile unsigned int  ui_BtnPortNumber;
	
	extern volatile unsigned int  uiSetIt_IsSet;
	
	extern volatile unsigned int uiMultiUse_uSTimer2;
	
	
	
	extern volatile unsigned int  ui_Encoder2_PortNumber;
	extern volatile unsigned int  ui_Encoder1_PortNumber;
	extern volatile unsigned int  ui_Encoder2_Port;
	extern volatile unsigned int  ui_Encoder1_Port;
	
	extern volatile unsigned long ul_LeftEncoder_Count;
	extern volatile unsigned long ul_RightEncoder_Count;
	
	extern volatile unsigned int  ui_Encoder3_PortNumber;
	extern volatile unsigned int  ui_Encoder4_PortNumber;
	extern volatile unsigned int  ui_Encoder3_Port;
	extern volatile unsigned int  ui_Encoder4_Port;
	
	extern volatile unsigned long ul_Encoder3_Count;
	extern volatile unsigned long ul_Encoder4_Count;
	
	//extern	volatile unsigned int  uc_test[100];
	


	
	
	
	void setEncoders(char b_Encoder1,char b_Encoder2);
	void setEncoders(char b_Encoder1,char b_Encoder2,char b_Encoder3,char b_Encoder4);
	void setIt();
	void set(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_ChaliePort4);
    void set(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3);	
	void setBtn(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_ChaliePort4,char b_PortBtnNumber);
    void setBtn(char b_ChaliePort1,char b_ChaliePort2,char b_ChaliePort3,char b_PortBtnNumber);		
	
	void CharliePlex();
	void Write(unsigned char uc_LED_Number,unsigned char uc_On_Off);
	void WriteMatrix(unsigned int ui_LED_Matrix);
	void Brightness(unsigned int ui_Display_Brightness);
	extern void (*MSE_DualPortedRam_Func)();
}



#endif
