
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "ssd1306.h"
#include "OLED.h"
#include "EEPROM.h"
#include "Modbus.h"

/* Variables ---------------------------------------------------------*/
extern uint8_t state; //TEMP

///выходы///
extern uint8_t Out_X1_1, Out_X1_2, Out_X1_3, Out_X1_4, Out_X1_5, Out_X1_6, Out_X1_7, Out_X1_8;//Устанавливаются в алгоритмах
extern uint8_t Out_X4_1, Out_X4_2, Out_X4_3, Out_X4_4, Out_X4_5, Out_X4_6, Out_X4_7, Out_X4_8;//Устанавливаются в алгоритмах

///токи///
extern int16_t I1_N;
extern int16_t I2_N;
extern int16_t I1_N_64ms;
///напряжения///
extern uint16_t V1_N, V2_N, V3_N; 
//extern uint16_t ;
extern uint8_t f_UART1;
extern uint8_t Regul_UpDown; //после длительного нажатия +-100

extern uint8_t F0_QF1; //cост-е выкл-ля
extern uint8_t F_I2C_Wr;
extern uint8_t f_VM_Con;

extern uint8_t f_DDC;
extern uint16_t PO_Ver_AFB;
extern uint32_t FAULT;
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //Settings , Protection      status,    Russian
//Line Test Device(LTD)       0m 
                                                     
#define Text0    {"   Status         "},{" AFB________      "}	// { 0x20,0x20,0x20,0x32,0x65,0x70,0x63,0xb8,0xc7,0x20,0x31,0x5f,0x30,0x20,0x20,0x20 }
#define Text1    {"  Protections     "},{"                  "}	// { 0x20,0x20,0x20,0x20,0x20,0x43,0xbf,0x61,0xbf,0x79,0x63,0x20,0x20,0x20,0x20,0x20 }
#define Text2    {"  Triggered       "},{"  protections     "}	// { 0x20,0x20,0x20,0x48,0x61,0x63,0xbf,0x70,0x6f,0xa6,0xba,0xb8,0x20,0x20,0x20,0x20 }
#define Text3    {"  Measurements    "},{"                  "}	// { 0x20,0x20,0x20,0xb1,0xb7,0xc3,0xba,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text4    {"  System state    "},{"                  "}	// { 0x43,0xbf,0x61,0xbf,0x79,0x63,0x20,0xa4,0x61,0xe6,0xb8,0xbf,0x20,0x20,0x20,0x20 }
#define Text5    {"  Statistics      "},{"                  "}	// { 0x20,0x43,0x70,0x61,0xb2,0x6f,0xbf,0x61,0xb3,0xc1,0xc3,0x65,0x20,0x20,0x20,0x20,0x20,0xb7,0x61,0xe6,0xb8,0xbf,0xc3,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }     
#define Text6    {"  Factory         "},{"                  "}	// { 0x20,0x43,0x6f,0x63,0xbf,0x6f,0xC7,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x42,0x78,0x6F,0xE3,0x2F,0x43,0xC3,0x78,0x68,0xE3,0x20,0x20,0x20,0x20,0x20 }
#define Text7    {"  Russian         "},{"                  "}	// { 0x20,0x43,0x6f,0x63,0xbf,0x6f,0xC7,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x43,0xB8,0x63,0xBF,0x65,0xBC,0xC3,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text8    {" Instantaneous+[ ]"},{" overcurrent  -[ ]"} 	//вкл-е защиты	// { 0x20,0xA5,0xB7,0xBC,0x65,0x70,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 } 
#define Text9    {" Overcurrent  +[ ]"},{" protection   -[ ]"}	// { 0x20,0x43,0xBF,0x61,0xBF,0xB8,0x63,0xBF,0xB8,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text10   {" Current rate +[ ]"},{" of rise prot.-[ ]"}	// { 0x20,0x4F,0x63,0xBD,0x6F,0xB3,0xBD,0xC3,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text11   {" Io   [ ] dI/dt[ ]"},{" Imax [ ] ExtTr[ ]"}	//сраб-е защиты	// { 0x20,0xA4,0x61,0xE6,0xB8,0xBF,0xC3,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text12   {"  Current         "},{"                  "}	// { 0x20,0x41,0xBD,0xB4,0xBB,0xB8,0xB9,0x63,0xBA,0xB8,0xB9,0x20,0x20,0x20,0x20,0x20 }
#define Text13   {"  Voltage         "},{"                  "}	// { 0x20,0x50,0x79,0x63,0x63,0xBA,0xB8,0xB9,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text14   {" Input/Output     "},{"  Status          "}
#define Text15   {" VM connection    "},{"                  "}
#define Text16   {" Number of SG     "},{" operations       "}	// { 0x58,0x31,0x20,0x42,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x58,0x31,0x20,0x42,0xC3,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text17   {" Current          "},{" correction       "}	// { 0x58,0x34,0x20,0x42,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x58,0x34,0x20,0x42,0xC3,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text18   {" Voltage          "},{" correction       "}	// { 0x20,0xA5,0x48,0x20,0xBE,0x6F,0xE3,0xBA,0xBB,0xC6,0xC0,0x65,0xBD,0x20,0x20,0x20 }
#define Text19   {" +Io   =     A    "},{" -Io   =     A    "}	// { 0x20,0x54,0x6F,0xBA,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text20   {" +Imax =     A    "},{"    dt =    ms    "}	// { 0x20,0x48,0x61,0xBE,0x70,0xC7,0xB6,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text21   {" +dI/dt=   0 A/ms "},{" -dI/dt=   0 A/ms "}	// { 0xA5,0xB7,0xBC,0x65,0x70,0x65,0xBD,0xB8,0x65,0x20,0xBB,0xB8,0xBD,0xB8,0xB8,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text22   {" TA1:  I=     A   "},{" TA2:  I=     A   "}	// { 0x4B,0x6F,0xBB,0xB8,0xC0,0x65,0x63,0xBF,0xB3,0x6F,0x20,0x20,0x20,0x20,0x20,0x20,0xBA,0x6F,0xBC,0xBC,0x79,0xBF,0x61,0xE5,0xB8,0xB9,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text23   {" C1:1  U=     V   "},{" C1:2  U=     V   "}	// { 0x20,0xA9,0x63,0xBF,0x61,0xB3,0xBA,0x61,0x20,0xA5,0x48,0xA4,0x20,0x20,0x20,0x20,0x20,0x20,0x52,0x6C,0x74,0x64,0x3D,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text24   {" X1 In            "},{" X1 Out           "}	// { 0x54,0x6F,0xBA,0x6F,0xB3,0x61,0xC7,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x6F,0xBF,0x63,0x65,0xC0,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text25   {" Io TA1       A   "},{" Io TA2       A   "}	// { 0x4D,0x61,0xBA,0x63,0xB8,0xBC,0x61,0xBB,0xBE,0x61,0xC7,0x20,0x20,0x20,0x20,0x20,0xBF,0x6F,0xBA,0x6F,0xB3,0x61,0xC7,0x20,0xB7,0x61,0xE6,0xB8,0xBF,0x61,0x20,0x20 }
#define Text26   {" Koef U1      %   "},{" Koef U2      %   "}	// { 0xB7,0x61,0xE6,0xB8,0xBF,0x61,0x20,0xBE,0x6F,0x20,0x64,0x49,0x2F,0x64,0x74,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text27   {" -Imax =  00 A    "},{"    dt =    ms    "}	//char Text27 []={"TA1:  I=0000A   TA2:  I=0000A   "};
#define Text28   {" X4 In            "},{" X4 Out           "}	// X4 In           X4 Out          char Text28 []={"C1:1  U=0000V   C1:2  U=0000V   "};

#define Text29   {" Feed  U=     V   "},{"                  "}	//char Text29 []={"      U=0000V                   "};
#define Text30   {" +Io   =  00A     "},{"  -Io   =  00A    "}	//char Text30 []={"+Io     00A     -Io     00A     "};
#define Text31   {" +Imax =  00A     "},{"    dt =    ms    "}	//
#define Text32   {" -Imax =  00A     "},{"    dt =    ms    "}	//
#define Text33   {" +dI/dt=   0A/ms  "},{"    dt =    ms    "}	//
#define Text34   {" -dI/dt=   0A/ms  "},{"    dt =    ms    "}	//

#define Text35   {"Fault:            "},{"                  "}	// { 0x4F,0xC1,0xB8,0xB2,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }

#define Text36   {"   AFB ON         "},{" Current 000000   "}	// { 0x20,0x20,0x20,0x41,0x46,0x42,0x20,0x42,0xBA,0xBB,0xC6,0xC0,0x65,0xBD,0x20,0x20,0x20,0x54,0x6F,0xBA,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text37   {"   Save           "},{"                  "}	// { 0x20,0x43,0x6F,0x78,0x70,0x61,0xBD,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text38   {"   Factory        "},{"                  "}	// { 0x20,0xA4,0x61,0xB3,0x6F,0xE3,0x63,0xBA,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text39   {" Current          "},{"  correction      "}	// { 0x20,0x4B,0x6F,0x70,0x65,0xBA,0xE5,0xB8,0xC7,0x20,0x54,0x6F,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 }
#define Text40   {" Voltage          "},{"  correction      "}	// { 0x20,0x4B,0x6F,0x70,0x65,0xBA,0xE5,0xB8,0xC7,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xBD,0x61,0xBE,0x70,0xC7,0xB6,0x65,0xBD,0xB8,0xC7,0x20,0x20,0x20,0x20,0x20 }
#define Text41   {" Io TA1       A   "},{"  Io TA2       A  "}	//
#define Text42   {" Koef U1      %   "},{"  Koef U2      %  "}	//
#define Text43   {" Koef U3      %   "},{"                  "}	//
#define Text44   {" STATUS AFB       "},{"  select          "}	//25_08_20
#define Text45   {" STATUS AFB_      "},{"                  "} //25_08_20
#define Text46   {" Various setup    "},{"                  "}	//25_08_20
#define Text47   {" Pressure         "},{"  control         "}	//25_08_20
//#define Text48   {" STATUS AFB_K                  "}//25_08_20

 // Русский язык 

 
char Text [96][18]={ Text0, Text1, Text2, Text3, Text4, Text5, Text6, Text7, Text8, Text9, Text10, Text11, Text12, Text13, Text14, Text15, Text16, Text17, Text18, Text19, Text20,
  Text21, Text22, Text23, Text24, Text25, Text26, Text27, Text28, Text29, Text30, Text31, Text32, Text33, Text34, Text35, Text36, Text37, Text38, Text39, Text40, Text41, Text42, 
  Text43, Text44, Text45, Text46, Text47
  /*,{ 0x20,0x20,0x20,0x32,0x65,0x70,0x63,0xb8,0xc7,0x20,0x31,0x5f,0x30,0x20,0x20,0x20 },
  { 0x20,0x20,0x20,0x20,0x20,0x43,0xbf,0x61,0xbf,0x79,0x63,0x20,0x20,0x20,0x20,0x20 },
  { 0x20,0x20,0x20,0x48,0x61,0x63,0xbf,0x70,0x6f,0xa6,0xba,0xb8,0x20,0x20,0x20,0x20 },
  { 0x20,0x20,0x20,0xb1,0xb7,0xc3,0xba,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 },
  { 0x43,0xbf,0x61,0xbf,0x79,0x63,0x20,0xa4,0x61,0xe6,0xb8,0xbf,0x20,0x20,0x20,0x20 },
  { 0x20,0x43,0x70,0x61,0xb2,0x6f,0xbf,0x61,0xb3,0xc1,0xc3,0x65,0x20,0x20,0x20,0x20,0x20,0xb7,0x61,0xe6,0xb8,0xbf,0xc3,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 },
  { 0x20,0x43,0x6f,0x63,0xbf,0x6f,0xC7,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x42,0x78,0x6F,0xE3,0x2F,0x42,0xC3,0x78,0x6f,0xE3,0x20,0x20,0x20,0x20 },
  { 0x20,0x43,0x6f,0x63,0xbf,0x6f,0xC7,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x43,0xB8,0x63,0xBF,0x65,0xBC,0xC3,0x20,0x20,0x20,0x20,0x20,0x20,0x20 },
  { 0x20,0xA5,0xB7,0xBC,0x65,0x70,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20 },
  { 0x20,0x43,0xBF,0x61,0xBF,0xB8,0x63,0xBF,0xB8,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x4F,0x63,0xBD,0x6F,0xB3,0xBD,0xC3,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0xA4,0x61,0xE6,0xB8,0xBF,0xC3,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x20,0x20,0x41,0xBD,0xB4,0xBB,0xB8,0xB9,0x63,0xBA,0xB8,0xB9,0x20,0x20,0x20 },
  { 0x20,0x20,0x20,0x50,0x79,0x63,0x63,0xBA,0xB8,0xB9,0x20,0x20,0x20,0x20,0x20,0x20 },
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x58,0x31,0x20,0x42,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x58,0x31,0x20,0x42,0xC3,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x58,0x34,0x20,0x42,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x58,0x34,0x20,0x42,0xC3,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0xA5,0x48,0x20,0xBE,0x6F,0xE3,0xBA,0xBB,0xC6,0xC0,0x65,0xBD,0x20,0x20,0x20 },
  { 0x20,0x54,0x6F,0xBA,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x48,0x61,0xBE,0x70,0xC7,0xB6,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0xA5,0xB7,0xBC,0x65,0x70,0x65,0xBD,0xB8,0x65,0x20,0xBB,0xB8,0xBD,0xB8,0xB8,0x20,0x20,0x52,0xB8,0xBA,0xB7,0x3D,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x6D,0x20,0x00 },
  { 0x4B,0x6F,0xBB,0xB8,0xC0,0x65,0x63,0xBF,0xB3,0x6F,0x20,0x20,0x20,0x20,0x20,0x20,0xBA,0x6F,0xBC,0xBC,0x79,0xBF,0x61,0xE5,0xB8,0xB9,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x20,0xA9,0x63,0xBF,0x61,0xB3,0xBA,0x61,0x20,0xA5,0x4B,0xA4,0x20,0x20,0x20,0x20,0x20,0x52,0xB8,0xBA,0xB7,0x3D,0x20,0x20,0x20,0x20,0x20,0x20,0x6D,0x20,0x00 },
  { 0x54,0x6F,0xBA,0x6F,0xB3,0x61,0xC7,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x6F,0xBF,0x63,0x65,0xC0,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x4D,0x61,0xBA,0x63,0xB8,0xBC,0x61,0xBB,0xC4,0xBD,0x61,0xC7,0x20,0x20,0x20,0x20,0xBF,0x6F,0xBA,0x6F,0xB3,0x61,0xC7,0x20,0xB7,0x61,0xE6,0xB8,0xBF,0x61,0x20,0x00 },
  { 0xA4,0x61,0xE6,0xB8,0xBF,0x61,0x20,0xBE,0x6F,0x20,0x64,0x49,0x2F,0x64,0x74,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x4F,0xC1,0xB8,0xB2,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x20,0x20,0x41,0x46,0x42,0x20,0x42,0xBA,0xBB,0xC6,0xC0,0x65,0xBD,0x20,0x20,0x20,0x54,0x6F,0xBA,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x43,0x6F,0x78,0x70,0x61,0xBD,0x65,0xBD,0xB8,0x65,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0xA4,0x61,0xB3,0x6F,0xE3,0x63,0xBA,0xB8,0x65,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x4B,0x6F,0x70,0x70,0x65,0xBA,0xE5,0xB8,0xC7,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xBF,0x6F,0xBA,0x61,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00 },
  { 0x20,0x4B,0x6F,0x70,0x70,0x65,0xBA,0xE5,0xB8,0xC7,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xBD,0x61,0xBE,0x70,0xC7,0xB6,0x65,0xBD,0xB8,0xC7,0x20,0x20,0x20,0x20,0x00 }, 
  { 0x20,0x20},
  { 0x20,0x20},
  { 0x20,0x20} */
}; //28_08_20


//char Faults [32][4]={{0x30,0x31,0x3B,0},{0x30,0x32,0x3B,0},{0x30,0x33,0x3B,0},{0x30,0x34,0x3B,0},{0x30,0x35,0x3B,0},{0x30,0x36,0x3B,0},{0x30,0x37,0x3B,0},
//{0x30,0x38,0x3B,0},{0x30,0x39,0x3B,0},{0x31,0x30,0x3B,0},{0x31,0x31,0x3B,0},{0x31,0x32,0x3B,0},{0x31,0x33,0x3B,0},{0x31,0x34,0x3B,0},{0x31,0x35,0x3B,0},
//{0x31,0x36,0x3B,0},{0x31,0x37,0x3B,0},{0x31,0x38,0x3B,0},{0x31,0x39,0x3B,0},{0x32,0x30,0x3B,0},{0x32,0x31,0x3B,0},{0x32,0x32,0x3B,0},{0x32,0x33,0x3B,0},
//{0x32,0x34,0x3B,0},{0x32,0x35,0x3B,0},{0x32,0x36,0x3B,0},{0x32,0x37,0x3B,0},{0x32,0x38,0x3B,0},{0x32,0x39,0x3B,0},{0x33,0x30,0x3B,0},{0x33,0x31,0x3B,0},{0x33,0x32,0x3B,0}};//{{0x30,0x31,0x3B},{0x30,0x32,0x3B},{0x30,0x33,0x3B}};

char Faults [32][4]={{0x30,0x31,0x3B,0},{0x30,0x32,0x3B,0},{0x30,0x33,0x3B,0},{0x30,0x34,0x3B,0},{0x30,0x35,0x3B,0},{0x30,0x36,0x3B,0},{0x30,0x37,0x3B,0},
{0x30,0x38,0x3B,0},{0x30,0x39,0x3B,0},{0x31,0x30,0x3B,0},{0x31,0x31,0x3B,0},{0x31,0x32,0x3B,0},{0x31,0x33,0x3B,0},{0x31,0x34,0x3B,0},{0x31,0x35,0x3B,0},
{0x31,0x36,0x3B,0},{0x31,0x37,0x3B,0},{0x31,0x38,0x3B,0},{0x31,0x39,0x3B,0},{0x32,0x30,0x3B,0},{0x32,0x31,0x3B,0},{0x32,0x32,0x3B,0},{0x32,0x33,0x3B,0},
{0x32,0x34,0x3B,0},{0x32,0x35,0x3B,0},{0x32,0x36,0x3B,0},{0x32,0x37,0x3B,0},{0x32,0x38,0x3B,0},{0x32,0x39,0x3B,0},{0x33,0x30,0x3B,0},{0x33,0x31,0x3B,0},{0x33,0x32,0x3B,0}};

uint16_t P_dIdt_Set_50mks = 100;
uint16_t N_dIdt_Set_50mks = 50;
uint16_t LTD_Izmer = 22;


uint16_t T_LCD_E = 10000; 	//задержка для подсветки
#define TT_LCD_E 20000   //Заданный интервал в циклах по 10мсек на откл-е подсветки экрана
   
uint8_t Ch_Knob_Set=0;
uint8_t Menu_Set = 0;
char IO_Menu[9] = {0,0,0,0,0,0,0,0,0};
char Var_Menu[7];
uint8_t Ch_Menu = 0;
uint8_t Menu_N[]={0,0,0,0}; //номер меню temp
uint8_t Menu_Stolbec=0;
uint8_t f_Rjd_Zdv=1;    //флаг Ряд сдвиг
uint8_t f_Stolb_Zdv=1;  //флаг Столбец сдвиг
uint8_t f_LCD_Ref=0;    //флаг обновление экрана
uint8_t Channel_Set=0;   //канал настройки
uint8_t F_Save = 0; //флаг сохранение уставок
uint8_t Language=0;

//флаги сработавших защит - названия?
extern uint8_t F_Over_Current;

uint8_t f_Imax_Srabot=0;//F_Zash_Imax

extern uint8_t Otkaz_IKZ; //F_Zash_LTD не прошло проверку 
extern uint8_t Funk_Otkl;

extern uint16_t V1_Koeff;
extern uint16_t V2_Koeff;
extern uint16_t V3_Koeff;
extern uint8_t  Fault_Menu;
extern uint8_t  f_Test;
uint16_t LTD_Threshold = 0;
uint8_t  Puk = 0;
uint8_t  N_Ekr = 0;
uint8_t  Migan = 0;
uint8_t B_Menu [29][5] = { 		//навигация  //<1/^2/>3/v4/*5
  {0,0,0,1,0},     {1,0,8,2,0},     {2,1,11,3,0},    {3,2,12,4,0},    {4,3,14,5,0},     {5,4,16,6,0}, 		// 0 1 2 3 4 5
  {6,5,17,6,0},    {0,0,0,0,0},     {1,8,19,9,1},    {1,8,20,10,2},   {1,9,21,10,3},    {2,11,11,11,4},		// 6 7 8 9 10 11
  {3,12,22,13,0},  {3,12,23,13,0},  {4,14,24,15,0},  {4,14,15,14,5},  {5,16,16,16,6},   {6,17,25,18,0},		// 12 13 14 15 16 17
  {6,17,26,18,0},  {8,19,19,19,7},  {9,20,27,20,8},  {10,21,21,21,9}, {12,22,22,22,10}, {13,23,23,23,11}, 	// 18 19 20 21 22 23
 {14,24,28,24,12}, {17,25,25,25,13}, {18,26,26,26,14}, {20,27,27,27,15}, {24,28,28,28,16}}; 					// 24 25 26 27 28 

/* Functions ---------------------------------------------------------------*/

void Menu_(void)
{ 
	V1_Koeff = (568 *(1000+(int8_t)K_V1))/1000; //27_05_20 перенес 
	V2_Koeff = (568 *(1000+(int8_t)K_V2))/1000;
	//V3_Koeff = (568 *(1000+(int8_t)K_V3))/1000;
	
	char retVal;
	LCD_Podsvetka( );
 ///////////////////////////////////////////////////////////////////////// 
	//if (Ch_Menu == 175) ssd1306_Init( ); //TEMP
    
	if (Ch_Menu == 195)
	{
	  P_dIdt_Set_50mks = (P_dIdt_Set)/(20); //*P_dIdt_TSet
	  N_dIdt_Set_50mks = (N_dIdt_Set)/(20); //*N_dIdt_TSet
	  
   //Тестовый режим
	  if((f_Test==1) && (Puk ==0))
	  {
		Menu_N[0]=1; Menu_N[1]=2; Menu_N[2]=0; Menu_N[3]=0; f_Rjd_Zdv=0; f_Stolb_Zdv=0; Menu_Stolbec=1;
	  }
	  
	  if((f_Test==0) && (Puk ==1))
	  {
		Menu_N[0]=0; Menu_N[1]=0; Menu_N[2]=0; Menu_N[3]=0; f_Rjd_Zdv=0; Menu_Stolbec=0;
	  }
	  Puk = f_Test;
    
 //ограничение значений
    #ifdef AFB_40   //28_09_20
    if((STATUS==1)||(STATUS==4)){ 
      if (P_Io_Set  > 8000) P_Io_Set = 8000;
      if (N_Io_Set  > 8000) N_Io_Set = 8000; 
      if (P_Imax_Set > 8000)P_Imax_Set = 8000;
      if (N_Imax_Set > 8000)N_Imax_Set = 8000;
      if (P_dIdt_Set  > 4000)P_dIdt_Set = 4000; 
      if (N_dIdt_Set  > 4000)N_dIdt_Set = 4000;}
    else {
      if (P_Io_Set  > 10000) P_Io_Set = 10000; //30_11_22 
      if (N_Io_Set  > 7500) N_Io_Set = 7500; 
      if (P_Imax_Set  > 10000)P_Imax_Set = 10000; //30_11_22 
      if (N_Imax_Set > 7500)N_Imax_Set = 7500;
      if (P_dIdt_Set  > 8000)P_dIdt_Set = 8000; 
      if (N_dIdt_Set  > 4000)N_dIdt_Set = 4000;}
    #else
    if((STATUS==1)||(STATUS==4)){ 
      if (P_Io_Set  > 4000) P_Io_Set = 4000;
      if (N_Io_Set  > 4000) N_Io_Set = 4000; 
      if (P_Imax_Set > 4000)P_Imax_Set = 4000; 
      if (N_Imax_Set > 4000)N_Imax_Set = 4000;
      if (P_dIdt_Set  > 2000)P_dIdt_Set = 2000; 
      if (N_dIdt_Set  > 2000)N_dIdt_Set = 2000;}
    else {
      if (P_Io_Set  > 10000) P_Io_Set = 10000; //30_11_22 
      if (N_Io_Set  > 3750) N_Io_Set = 3750; 
      if (P_Imax_Set > 7000)P_Imax_Set = 7000;
      if (N_Imax_Set > 3500)N_Imax_Set = 3500;
      if (P_dIdt_Set  > 8000)P_dIdt_Set = 8000; 
      if (N_dIdt_Set  > 2000)N_dIdt_Set = 2000;}
    #endif //28_09_20)
    
    //29_07_21 - ограничение уставки по минимуму
    if (P_Io_Set  < 400) P_Io_Set = 400;
    if (N_Io_Set  < 400) N_Io_Set = 400; 
    if (P_Imax_Set < 400)P_Imax_Set = 400;
    if (N_Imax_Set < 400)N_Imax_Set = 400;
    if (P_dIdt_Set < 1000)P_dIdt_Set = 1000; 
    if (N_dIdt_Set < 1000)N_dIdt_Set = 1000;
    
    if (N_Imax_TSet > 90) N_Imax_TSet = 90;
    if (N_Imax_TSet < 1) N_Imax_TSet = 1;
    if (P_Imax_TSet > 90) P_Imax_TSet = 90;
    if (P_Imax_TSet < 1) P_Imax_TSet = 1;
      
    if ((int16_t)K_V1 < (int8_t)(-125)) K_V1 =(-126);
    if ((int16_t)K_V1 > (int8_t)( 126)) K_V1 =( 127);
    if ((int16_t)K_V2 < (int8_t)(-125)) K_V2 =(-126);
    if ((int16_t)K_V2 > (int8_t)( 126)) K_V2 =( 127);
    //if ((int16_t)K_V3 < (int8_t)(-125)) K_V3 =(-126);
    //if ((int16_t)K_V3 > (int8_t)( 126)) K_V3 =( 127);
    
    //LTD_Threshold = LTD_Thresh/10; //для согласования с панелькой
    //if(LTD_Threshold < 50) {LTD_Threshold = 50; LTD_Thresh = 500;}
    //if(LTD_Threshold > 3200) {LTD_Threshold = 3200; LTD_Thresh = 32000;}
	
   
    /*if(((STATUS==2) || (STATUS==5))==0) Trig_Prot|=0x0008;*/
	 if((STATUS !=2)&&(STATUS !=5)){ //в другой версии файла
      Trig_Prot|=0x0008; Trig_Prot|=0x0010;
    }
  }
  
  //обновление экрана раз в 2сек
  if (((Ch_Menu > 200) || f_LCD_Ref == 1 )&&((FAULT == 0) || Fault_Menu || f_Test))
  {
	Ch_Menu = 0;
    f_LCD_Ref = 0;
    //if((Menu_N[0] == 1)&&(Menu_N[1] == 2)&&(Menu_Stolbec == 1)) state =1;  //25_03_24
    //else state = 0;
	ssd1306_Fill(Black);
	ssd1306_SetCursor(1, 5);
	retVal = ssd1306_WriteString(Text [N_Ekr*2], Font_7x10, White);  //Font_11x18
	ssd1306_SetCursor(1, 22);
	  //Font_11x18
	if(((N_Ekr == 1)||(N_Ekr == 6)) && F_Save)
	  retVal = ssd1306_WriteString(Text [37*2], Font_7x10, White); //temp
	else 
	  retVal = ssd1306_WriteString(Text [N_Ekr*2 + 1], Font_7x10, White);
	Otobr_Znach (0);
	//ssd1306_SetCursor(20, 5); retVal = ssd1306_WriteString(Text [N_Ekr][1], Font_11x18, White);
	ssd1306_UpdateScreen( );
  } 
  Ch_Menu++;
  
  if((FAULT != 0) && (Ch_Menu > 200) && ((Fault_Menu || f_Test)==0)) //отображение ошибки
  {
    Ch_Menu = 0;
	ssd1306_Fill(Black);
	ssd1306_SetCursor(4, 5);
	retVal = ssd1306_WriteString(Text [70], Font_7x10, White); //Font_11x18
	
	
    //TM_HD44780_Puts(0, 0, Text[35+Language] );
    uint8_t ii = 0;
    uint8_t x = 0;
    while (ii < 32)
    {
      if((FAULT >> ii) & 1)
      {
        if( x < 3 )
		  ssd1306_SetCursor((47 + (x * 23)), 5); //33
          //TM_HD44780_Puts((7+x*3), 0, Faults[ii] );
        else if( (x > 2)&&(x < 8) )
		  ssd1306_SetCursor((5 + ((x-3) * 23)), 22);
		else 
		  ssd1306_SetCursor((5 + ((x-8) * 23)), 41);
		  //TM_HD44780_Puts(((x-3)*3), 1, Faults[ii] );
        x++;
		//номера ошибок в Faults[ii] убрать пробел или запятую?
		retVal = ssd1306_WriteString(Faults[ii], Font_7x10, White); //
		//ssd1306_UpdateScreen( );
      }
      ii++;
    }
	ssd1306_UpdateScreen( );
  }
  
  if((F0_QF1==1) && (Ch_Menu > 200)) //обновление экрана при включенном AFB
  {
    Ch_Menu=0;
    //TM_HD44780_Puts(0, 0, Text[36+Language] );
    Var_Displey (I1_N_64ms/10); 
	//TM_HD44780_Puts(8, 1, Var_Menu );  // токи LEM-датчиков
	ssd1306_SetCursor(25, 60);
	retVal = ssd1306_WriteString(Var_Menu, Font_11x18, White); //???
	ssd1306_UpdateScreen( );
  }
  
}

/// для отображения переменных
 void Var_Displey (int16_t Var_State)
{       
    if (Var_State & 0x8000) 
    {
      Var_State=(~Var_State);  Var_State++;
      Var_Menu[0]=0x2D;
      Var_Menu[1]=(Var_State/1000)|48;
      Var_Menu[2]=((Var_State%1000)/100)|48;
      Var_Menu[3]=((Var_State%100)/10)|48;
      Var_Menu[4]=(Var_State%10)|48;
      Var_Menu[5]=0;
      //Var_Menu[6]=0x20;
    } 
    else 
    { 
      Var_Menu[0]=0x20; //0x2B; '+'
      Var_Menu[1]=(Var_State/1000)|48;
      Var_Menu[2]=((Var_State%1000)/100)|48;
      Var_Menu[3]=((Var_State%100)/10)|48;
      Var_Menu[4]=(Var_State%10)|48;
      Var_Menu[5]=0; 
    }
}

/// для отображения переменных 8бит
 void Var_Disp8 (int8_t Var_State)
{       
    if (Var_State & 0x80) 
    {
      Var_State=~Var_State;  
      Var_Menu[0]=0x2D;
      Var_Menu[1]=(Var_State/10)|48;
      Var_Menu[2]=(Var_State%10)|48;
      Var_Menu[3]=0;
      //Var_Menu[6]=0x20;
    } 
    else 
    { 
      Var_Menu[0]=0x2B; //0x2B; '+'
      Var_Menu[1]=(Var_State/10)|48;
      Var_Menu[2]=(Var_State%10)|48;
      Var_Menu[3]=0; 
    }
}

/// для отображения  больших переменных
 void Var_Displey_2 (int16_t Var_State)
{       
    if (Var_State & 0x8000) 
    {
      Var_State=(~Var_State);  Var_State++;
      Var_Menu[0]=0x2D;
      Var_Menu[1]=(Var_State/10000)|48;
      Var_Menu[2]=((Var_State%10000)/1000)|48;
      Var_Menu[3]=((Var_State%1000)/100)|48;
      Var_Menu[4]=((Var_State%100)/10)|48;
      Var_Menu[5]=(Var_State%10)|48;
      Var_Menu[6]=0;
      //Var_Menu[6]=0x20;
    } 
    else 
    { 
      Var_Menu[0]=0x20; //0x2B; '+'
      Var_Menu[1]=(Var_State/10000)|48;
      Var_Menu[2]=((Var_State%10000)/1000)|48;
      Var_Menu[3]=((Var_State%1000)/100)|48;
      Var_Menu[4]=((Var_State%100)/10)|48;
      Var_Menu[5]=(Var_State%10)|48;
      Var_Menu[6]=0;
    }
}
/// для отображения состояния ввода-вывода
 void IO_Displey (uint8_t IO_Pin_State)
{  
      IO_Menu[0]= (IO_Pin_State & 0x80)>>7|48; 
      IO_Menu[1]= (IO_Pin_State & 0x40)>>6|48; 
      IO_Menu[2]= (IO_Pin_State & 0x20)>>5|48;
      IO_Menu[3]= (IO_Pin_State & 0x10)>>4|48; 
      IO_Menu[4]= (IO_Pin_State & 0x08)>>3|48; 
      IO_Menu[5]= (IO_Pin_State & 0x04)>>2|48;
      IO_Menu[6]= (IO_Pin_State & 0x02)>>1|48; 
      IO_Menu[7]= (IO_Pin_State & 0x01)|48;
      IO_Menu[8]= 0;
}
 

/// обработка кнопок
 void Knob_Prev (void) //1 //up
 {
   if(Ch_Knob_Set == 0)
   	N_Ekr = B_Menu [N_Ekr][0]; 
   Channel_Set = B_Menu [N_Ekr][4];
   f_LCD_Ref=1;
   //if(Menu_Stolbec && (Ch_Knob_Set==0))Menu_Stolbec--;
   T_LCD_E = TT_LCD_E; //задержка для подсветки
   //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); //Включаем подсветку экрана
 }

 void Knob_Uppe (void) //2 dow
 {
   if(Ch_Knob_Set == 0) N_Ekr = B_Menu [N_Ekr][1]; 
   Channel_Set = B_Menu [N_Ekr][4];
   if(Channel_Set != 0) Menu_Nastrojki (1);
   /*if (((Menu_Stolbec==0)&&F_Save)==0){
   if(Menu_N[Menu_Stolbec] && Ch_Knob_Set==0) Menu_N[Menu_Stolbec]--;
   if((f_Stolb_Zdv==0) || (Channel_Set!=0)) Menu_Nastrojki (1); // ||(Channel_Set!=0)
   //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); //Включаем подсветку экрана
   }*/
   f_LCD_Ref=1;
   T_LCD_E = TT_LCD_E; //задержка для подсветки
 }

 void Knob_Next (void) //3 //left
 {
   if(Ch_Knob_Set == 0)
	 N_Ekr = B_Menu [N_Ekr][2];  
   Channel_Set = B_Menu [N_Ekr][4];
   f_LCD_Ref=1; T_LCD_E = TT_LCD_E; //задержка для подсветки
   /*if (((Menu_Stolbec==0)&&F_Save)==0){
   if(f_Stolb_Zdv){Menu_Stolbec++;
   Menu_N[Menu_Stolbec]=0;}
   //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); //Включаем подсветку экрана
   }*/
 }

 void Knob_Down (void) //4 //right
 {
   if(Ch_Knob_Set == 0) N_Ekr = B_Menu [N_Ekr][3]; 
   Channel_Set = B_Menu [N_Ekr][4];
   if(Channel_Set != 0) Menu_Nastrojki (2);
   f_LCD_Ref=1; T_LCD_E = TT_LCD_E; //задержка для подсветки
   /*if (((Menu_Stolbec==0)&&F_Save)==0){
   f_LCD_Ref=1;
   if(f_Rjd_Zdv && Ch_Knob_Set==0)Menu_N[Menu_Stolbec]++;
   if((f_Stolb_Zdv==0) || (Channel_Set!=0)) Menu_Nastrojki (2);
   T_LCD_E = TT_LCD_E; //задержка для подсветки
   //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); //Включаем подсветку экрана
   }*/
 }

 void Knob_Set (void) //5
 { 
   Regul_UpDown = 10;     //регулировка +-10 ед.
   f_LCD_Ref = 1;         //обновл экрана
   Channel_Set = B_Menu [N_Ekr][4];
   if((Channel_Set == 7)||(Channel_Set == 8)||(Channel_Set == 15)||(Channel_Set == 9) ) // 7,8,15,9?
   {
     Ch_Knob_Set ++;//Menu_Set = Ch_Knob_Set; //Menu_Setting[Menu_N] + Ch_Knob_Set;
     //if(Channel_Set==6) Ch_Knob_Set++; //для LTD
	 if(Ch_Knob_Set > 2) {Ch_Knob_Set = 0; F_Save =1;} //Menu_Set=0;
   }
   
   if(((N_Ekr == 1)||(N_Ekr == 6))&&F_Save)
   {F_I2C_Wr = 1; F_Save = 0; }  //Ch_Knob_Set=0;
   
   T_LCD_E = TT_LCD_E; //задержка для подсветки
   //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); //Включаем подсветку экрана
   
   //if(Menu_N[0]>1){if(Menu_N[1]) Language=44; else Language=0;} //if(Language==0)выбор языка 16_05
 }

 /// настройки  /// настройки  /// настройки  /// настройки  /// настройки 
void Menu_Nastrojki (uint8_t Regul)
{
  //F_Save =1; //if((f_Stolb_Zdv==0) || (Channel_Set!=0)) Menu_Nastrojki (1);
  switch (Channel_Set) {
  case 0:  break;
  case 7: 
	if(Ch_Knob_Set == 1) 
	{ //28_09_20
	  #ifdef AFB_40  
	  if((STATUS==1)||(STATUS==4)){  
		if (Regul==1) {P_Io_Set += Regul_UpDown; if(P_Io_Set > 8000) P_Io_Set = 8000; }
		if (Regul==2) {P_Io_Set -= Regul_UpDown; if(P_Io_Set < 400) P_Io_Set = 400;}
	  }
	  else {
		if (Regul==1) {P_Io_Set += Regul_UpDown; if(P_Io_Set > 10000) P_Io_Set = 10000; } // 30_11_22 
		if (Regul==2) {P_Io_Set -= Regul_UpDown; if(P_Io_Set < 400) P_Io_Set = 400;}
	  }
	  #else
	  if((STATUS==1)||(STATUS==4)){  
		if (Regul==1) {P_Io_Set += Regul_UpDown; if(P_Io_Set > 4000) P_Io_Set = 4000; }
		if (Regul==2) {P_Io_Set -= Regul_UpDown; if(P_Io_Set < 400) P_Io_Set = 400;}
	  }
	  else {
		if (Regul==1) {P_Io_Set += Regul_UpDown; if(P_Io_Set > 12000) P_Io_Set = 12000; } // 30_11_22 
		if (Regul==2) {P_Io_Set -= Regul_UpDown; if(P_Io_Set < 400) P_Io_Set = 400;}
	  }
	  #endif
	}  
	if(Ch_Knob_Set == 2) 
	{ 
	  #ifdef AFB_40 
	  if((STATUS==1)||(STATUS==4)){  
		if (Regul==1) {N_Io_Set += Regul_UpDown; if(N_Io_Set > 8000) N_Io_Set = 8000; }
		if (Regul==2) {N_Io_Set -= Regul_UpDown; if(N_Io_Set < 400) N_Io_Set = 400;}
	  }
	  else {
		if (Regul==1) {N_Io_Set += Regul_UpDown; if(N_Io_Set > 7500) N_Io_Set = 7500; }
		if (Regul==2) {N_Io_Set -= Regul_UpDown; if(N_Io_Set < 400) N_Io_Set = 400; }
	  }
	  #else
	  if((STATUS==1)||(STATUS==4)){  
		if (Regul==1) {N_Io_Set += Regul_UpDown; if(N_Io_Set > 4000) N_Io_Set = 4000; }
		if (Regul==2) {N_Io_Set -= Regul_UpDown; if(N_Io_Set < 500) N_Io_Set = 500;}
	  }
	  else {
		if (Regul==1) {N_Io_Set += Regul_UpDown; if(N_Io_Set > 8000) N_Io_Set = 8000; }
		if (Regul==2) {N_Io_Set -= Regul_UpDown; if(N_Io_Set < 500) N_Io_Set = 500; }
	  } 
	#endif  
	}  break;
  
  case 8: if(Ch_Knob_Set==1) 
  { //28_09_20
    #ifdef AFB_40  
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1) {P_Imax_Set += Regul_UpDown; if(P_Imax_Set > 8000) P_Imax_Set = 8000; }
      if (Regul==2) {P_Imax_Set -= Regul_UpDown; if(P_Imax_Set < 400) P_Imax_Set = 400;}
    }
    else {
      if (Regul==1) {P_Imax_Set += Regul_UpDown; if(P_Imax_Set > 10000) P_Imax_Set = 10000; } // 30_11_22 
      if (Regul==2) {P_Imax_Set -= Regul_UpDown; if(P_Imax_Set < 400) P_Imax_Set = 400;}
    }
    #else
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1) {P_Imax_Set += Regul_UpDown; if(P_Imax_Set > 8000) P_Imax_Set = 8000; }
      if (Regul==2) {P_Imax_Set -= Regul_UpDown; if(P_Imax_Set < 500)  P_Imax_Set = 500;}
    }
    else {
      if (Regul==1) {P_Imax_Set += Regul_UpDown; if(P_Imax_Set > 12000) P_Imax_Set = 12000; }
      if (Regul==2) {P_Imax_Set -= Regul_UpDown; if(P_Imax_Set < 500)  P_Imax_Set = 500;}
    }
    #endif
    //if (Regul==1) {P_Imax_Set += Regul_UpDown; if(P_Imax_Set > 7000)P_Imax_Set = 7000;}
    //if (Regul==2) {P_Imax_Set -= Regul_UpDown; if(P_Imax_Set & 0x8000)P_Imax_Set = 0;} 
    
  }  
  if(Ch_Knob_Set==2) 
  { 
     if((Regul==1) && (P_Imax_TSet < 99))P_Imax_TSet ++; 
     if((Regul==2) && (P_Imax_TSet > 1))P_Imax_TSet --;
  }  break;
  
  case 15: if(Ch_Knob_Set==1) 
  { //28_09_20
    #ifdef AFB_40  
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1) {N_Imax_Set += Regul_UpDown; if(N_Imax_Set > 8000) N_Imax_Set = 8000; }
      if (Regul==2) {N_Imax_Set -= Regul_UpDown; if(N_Imax_Set < 400) N_Imax_Set = 400;}
    }
    else {
      if (Regul==1) {N_Imax_Set += Regul_UpDown; if(N_Imax_Set > 7500) N_Imax_Set = 7500; }
      if (Regul==2) {N_Imax_Set -= Regul_UpDown; if(N_Imax_Set < 400) N_Imax_Set = 400;}
    }
    #else
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1) {N_Imax_Set += Regul_UpDown; if(N_Imax_Set > 8000) N_Imax_Set = 8000; }
      if (Regul==2) {N_Imax_Set -= Regul_UpDown; if(N_Imax_Set < 500) N_Imax_Set = 500;}
    }
    else {
      if (Regul==1) {N_Imax_Set += Regul_UpDown; if(N_Imax_Set > 8000) N_Imax_Set = 8000; }
      if (Regul==2) {N_Imax_Set -= Regul_UpDown; if(N_Imax_Set < 500) N_Imax_Set = 500;}
    }
    #endif
    /*if (Regul==1) {N_Imax_Set += Regul_UpDown; if(N_Imax_Set > 3500)N_Imax_Set = 3500;}
    if (Regul==2) {N_Imax_Set -= Regul_UpDown; if(N_Imax_Set & 0x8000)N_Imax_Set = 0;}*/
  }  
  if(Ch_Knob_Set==2) 
  { 
     if((Regul==1) && (N_Imax_TSet < 99))N_Imax_TSet ++; 
     if((Regul==2) && (N_Imax_TSet > 1))N_Imax_TSet --;
  }  break;
  
  case 9: if(Ch_Knob_Set==1) 
  { 
    
    #ifdef AFB_40  
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1)  {P_dIdt_Set += Regul_UpDown; if(P_dIdt_Set > 4000)P_dIdt_Set = 4000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {P_dIdt_Set -= Regul_UpDown; if((int16_t)P_dIdt_Set < 1000)P_dIdt_Set = 1000;  }
    }
    else {
      if (Regul==1)  {P_dIdt_Set += Regul_UpDown; if(P_dIdt_Set > 8000)P_dIdt_Set = 8000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);//17_11_21
      if (Regul==2)  {P_dIdt_Set -= Regul_UpDown; if((int16_t)P_dIdt_Set < 1000)P_dIdt_Set = 1000;  }
    }
    #else
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1)  {P_dIdt_Set += Regul_UpDown; if(P_dIdt_Set > 8000)P_dIdt_Set = 8000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {P_dIdt_Set -= Regul_UpDown; if((int16_t)P_dIdt_Set < 1000)P_dIdt_Set = 1000;  }
    }
    else {
      if (Regul==1)  {P_dIdt_Set += Regul_UpDown; if(P_dIdt_Set > 8000)P_dIdt_Set = 8000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);//17_11_21
      if (Regul==2)  {P_dIdt_Set -= Regul_UpDown; if((int16_t)P_dIdt_Set < 1000)P_dIdt_Set = 1000;  }
    }
    #endif
    
    //if (Regul==1)  {P_dIdt_Set += Regul_UpDown; if(P_dIdt_Set > 15000)P_dIdt_Set = 15000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
    //if (Regul==2)  {P_dIdt_Set -= Regul_UpDown; if(P_dIdt_Set < 10)P_dIdt_Set = 10;  }
  }  
  if(Ch_Knob_Set==2) 
  { 
	if((STATUS==1)||(STATUS==4)){  
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 4000) N_dIdt_Set = 4000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000) N_dIdt_Set = 1000;  }
    }
    else {
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 8000) N_dIdt_Set = 8000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);//17_11_21
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000)N_dIdt_Set = 1000;  }
    }
  }  break;
  
  /*case 5: if(Ch_Knob_Set==1)
  { 
    #ifdef AFB_40  
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 4000)N_dIdt_Set = 4000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000)N_dIdt_Set = 1000;  }
    }
    else {
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 4000)N_dIdt_Set = 4000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000)N_dIdt_Set = 1000;  }
    }
    #else
    if((STATUS==1)||(STATUS==4)){  
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 2000)N_dIdt_Set = 2000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000)N_dIdt_Set = 1000;  }
    }
    else {
      if (Regul==1)  {N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 2000)N_dIdt_Set = 2000;}//P_dIdt_Set_50mks = (P_dIdt_Set)/(20*P_dIdt_TSet);
      if (Regul==2)  {N_dIdt_Set -= Regul_UpDown; if((int16_t)N_dIdt_Set < 1000)N_dIdt_Set = 1000;  }
    }
    #endif
    
    //if(Regul==1) { N_dIdt_Set += Regul_UpDown; if(N_dIdt_Set > 2000)N_dIdt_Set = 2000; } //N_dIdt_Set_50mks = (N_dIdt_Set)/(20*N_dIdt_TSet);
    //if(Regul==2) { N_dIdt_Set -= Regul_UpDown; if(N_dIdt_Set < 10)N_dIdt_Set = 10;    }
    
  }  
  if(Ch_Knob_Set==2) 
  { Ch_Knob_Set=0; //20_09
     //if((Regul==1) && (N_dIdt_TSet < 90))N_dIdt_TSet ++; 
     //if((Regul==2) && (N_dIdt_TSet > 0))N_dIdt_TSet --;
  } break;*/

  /*case 6: if(Ch_Knob_Set){ 
    if(Regul==1) LTD_Threshold += Regul_UpDown; 
    if(Regul==2) LTD_Threshold -= Regul_UpDown;}  
    if(LTD_Threshold > 3200) LTD_Threshold = 3200; //
    if(LTD_Threshold < 50) LTD_Threshold = 50;
    //LTD_Thresh = LTD_Threshold * 10; // 18_10_19 
    break;*/
    
    //корректировочные значения 20,11
  /*case 7: if(Ch_Knob_Set==1) //
  { 
    if((Regul==1) && ((int8_t)Io_TA1 < (int8_t) 100)) Io_TA1 ++; 
    if((Regul==2) && ((int8_t)Io_TA1 > (int8_t)(-100)))  Io_TA1 --; 
  }  
  if(Ch_Knob_Set==2) 
  { 
    if((Regul==1) && ((int8_t)Io_TA2 < (int8_t) 50))  Io_TA2 ++; 
    if((Regul==2) && ((int8_t)Io_TA2 > (int8_t)(-50)))  Io_TA2 --;
  } break;
  
  case 8: if(Ch_Knob_Set==1){ //
    if((Regul==1) && ((int8_t)K_V1 < (int8_t)127))    {K_V1 ++; V1_Koeff= (568 *(1000+(int8_t)K_V1))/1000; } 
    if((Regul==2) && ((int8_t)K_V1 > (int8_t)(-127))) {K_V1 --; V1_Koeff= (568 *(1000+(int8_t)K_V1))/1000; } 
  }  
  if(Ch_Knob_Set==2){ 
    if((Regul==1) && ((int8_t)K_V2 < (int8_t)127))    {K_V2 ++; V2_Koeff= (568 *(1000+(int8_t)K_V2))/1000;} 
    if((Regul==2) && ((int8_t)K_V2 > (int8_t)(-127))) {K_V2 --; V2_Koeff= (568 *(1000+(int8_t)K_V2))/1000;} 
  } break;
  
  case 9: if(Ch_Knob_Set){ //06_03_25
    //if((Regul==1) && ((int8_t)K_V3 < (int8_t)127))    {K_V3 ++; V3_Koeff= (568 *(1000+(int8_t)K_V3))/1000;} 
    //if((Regul==2) && ((int8_t)K_V3 > (int8_t)(-127))) {K_V3 --; V3_Koeff= (568 *(1000+(int8_t)K_V3))/1000;} 
  } break;
  
  case 10: // вкл/выкл Imax
   if(Ch_Knob_Set==1){ 
    if(Regul==1) Trig_Prot|=0x0080; 
    if((Regul==2)) Trig_Prot&=0xff7f; // if((Regul==2)&&(((Trig_Prot&0x0002)||(STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xff7f;
  } 
   if(Ch_Knob_Set==2){ 
    if(Regul==1) Trig_Prot|=0x0020; 
    if((Regul==2)) Trig_Prot&=0xffdf; // if((Regul==2)&&(((Trig_Prot&0x0004)||(STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xffdf;
  } break;
  
  case 11: // изменение режима АФБ
    if(Ch_Knob_Set){ 
    if((Regul==1) && (STATUS < 5)) {STATUS ++;} //28_09_20
    if((Regul==2) && (STATUS > 0)) {STATUS --;} //28_08_20
  } break;
  
  case 12: //  вкл/выкл Iо
  if(Ch_Knob_Set==1){ 
    if(Regul==1) Trig_Prot|=0x0002; 
    if((Regul==2)&&(((STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xfffd; // if((Regul==2)&&(((Trig_Prot&0x0080)||(STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xfffd; 
  }
  if(Ch_Knob_Set==2){ //05_01_20
    if(Regul==1) Trig_Prot|=0x0004; 
    if((Regul==2)&&(((STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xfffb; // if((Regul==2)&&(((Trig_Prot&0x0020)||(STATUS==2)||(STATUS==5))!=0)) Trig_Prot&=0xfffb;
  } break;
  
  case 13: // вкл/выкл dI/dt 
    // 17_11_21
  if ((STATUS==2)||(STATUS==5)){
    if(Ch_Knob_Set==1){ 
      if(Regul==1) Trig_Prot|=0x0008; 
      if(Regul==2) Trig_Prot&=0xfff7; 
    }
    if(Ch_Knob_Set==2){ //05_01_20
      if(Regul==1) Trig_Prot|=0x0010; 
      if(Regul==2) Trig_Prot&=0xffef; 
  } }
  break;
  
  case 14: // вкл/выкл датчик давления
    if(Ch_Knob_Set){ 
    if(Regul==1) Trig_Prot|=0x0001; 
    if(Regul==2) Trig_Prot&=0xfffe;
  } break;*/
  
  default: break;
  
  }
}


void Otobr_Znach (uint8_t Channel_Znach) //uint8_t Channel_Znach
{ 
  char LCD_Tset []= {0,0,0};
  char Output_X1 []= {Out_X1_8|48, Out_X1_7|48, Out_X1_6|48, Out_X1_5|48, Out_X1_4|48, Out_X1_3|48, Out_X1_2|48, Out_X1_1|48,0};
  char Output_X4 []= {Out_X4_8|48, Out_X4_7|48, Out_X4_6|48, Out_X4_5|48, Out_X4_4|48, Out_X4_3|48, Out_X4_2|48, Out_X4_1|48,0};
  char Gal_k = ' ';
  //char Corr_set []= {0,0,0,0};
    
  switch (Channel_Set)
  {
  case 0:  break; //reserved  
  case 1:  ssd1306_SetCursor(113, 5);
  			if(Trig_Prot & 0x02)  Gal_k = 'v';  // Io+ //String  ssd1306_WriteChar(Gal_k, Font_7x10, White);
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
		    ssd1306_SetCursor(113, 22);
		    if(Trig_Prot & 0x04) Gal_k = 'v';	//{ ssd1306_WriteChar("[v]", Font_7x10, White);} // Io-
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White); 
			break;
	case 2:  ssd1306_SetCursor(113, 5);
  			if(Trig_Prot & 0x20)  Gal_k = 'v';  // Io+ //String  ssd1306_WriteChar(Gal_k, Font_7x10, White);
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
		    ssd1306_SetCursor(113, 22);
		    if(Trig_Prot & 0x40) Gal_k = 'v';	//{ ssd1306_WriteChar("[v]", Font_7x10, White);} // Io-
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White); 
			break;
	case 3:  ssd1306_SetCursor(113, 5);
  			if(Trig_Prot & 0x08)  Gal_k = 'v';  // Io+ //String  ssd1306_WriteChar(Gal_k, Font_7x10, White);
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
		    ssd1306_SetCursor(113, 22);
		    if(Trig_Prot & 0x10) Gal_k = 'v';	//{ ssd1306_WriteChar("[v]", Font_7x10, White);} // Io-
           	else Gal_k = ' ';
			ssd1306_WriteChar(Gal_k, Font_7x10, White); 
			break;
	case 4: ssd1306_SetCursor(49, 5); Gal_k = ' ';
	  		if(Status_Flag & 0x0006) Gal_k = 'v'; 
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
			ssd1306_SetCursor(49, 22); Gal_k = ' ';
	  		if(Status_Flag & 0x0180) Gal_k = 'v'; 
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
			ssd1306_SetCursor(98, 5); Gal_k = ' ';
	  		if(Status_Flag & 0x0018) Gal_k = 'v';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
			ssd1306_SetCursor(98, 22); Gal_k = ' ';
	  		if(Status_Flag & 0x0060) Gal_k = 'v';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
			break;
	case 5: ssd1306_SetCursor(98, 5); Gal_k = ' ';
			if(f_VM_Con == 0) Gal_k = 'v';
			ssd1306_WriteChar(Gal_k, Font_7x10, White);
			break;
	case 6: ssd1306_SetCursor(91, 22);
			Var_Displey (Count_Switch);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 7: ssd1306_SetCursor(49, 5);
			Var_Displey_2 (P_Io_Set);  //Var_Displey
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(49, 22);
			Var_Displey_2 (N_Io_Set);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 8: ssd1306_SetCursor(42, 5);
			Var_Displey_2 (P_Imax_Set);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			ssd1306_SetCursor(49, 22);
			LCD_Tset[0]=(P_Imax_TSet/10)|48; LCD_Tset[1]=(P_Imax_TSet%10)|48;
			ssd1306_WriteString(LCD_Tset, Font_7x10, White);
			break;
    case 15: ssd1306_SetCursor(42, 5);
			Var_Displey_2 (N_Imax_Set);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			ssd1306_SetCursor(49, 22);
			LCD_Tset[0]=(N_Imax_TSet/10)|48; LCD_Tset[1]=(N_Imax_TSet%10)|48;
			ssd1306_WriteString(LCD_Tset, Font_7x10, White);
			break;
	case 9: ssd1306_SetCursor(42, 5);
			Var_Displey_2 (P_dIdt_Set);
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(42, 22);
			Var_Displey_2 (N_dIdt_Set);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 10: ssd1306_SetCursor(35, 5);
			Var_Displey_2 (I1_N);
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			Var_Displey ((I2_N/10));
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 11: ssd1306_SetCursor(35, 5);
			Var_Displey (V1_N);
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			Var_Displey (V2_N);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 12: ssd1306_SetCursor(35, 5);
			IO_Displey (Input_X1);
			ssd1306_WriteString(IO_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			ssd1306_WriteString(Output_X1, Font_7x10, White);
			break;
	case 16: ssd1306_SetCursor(35, 5);
			IO_Displey (Input_X2);
			ssd1306_WriteString(IO_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			ssd1306_WriteString(Output_X4, Font_7x10, White);
			break;
	case 13: ssd1306_SetCursor(35, 5);
			Var_Disp8 (Io_TA1);
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			Var_Disp8 (Io_TA2);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	case 14: ssd1306_SetCursor(35, 5);
			Var_Displey (K_V1);
			ssd1306_WriteString(Var_Menu, Font_7x10, White); 
			ssd1306_SetCursor(35, 22);
			Var_Displey (K_V2);
			ssd1306_WriteString(Var_Menu, Font_7x10, White);
			break;
	
           	/*if(Trig_Prot & 0x18)TM_HD44780_PutCustom(14, 0, 0 ); //dI/dt
           	else TM_HD44780_PutCustom(14, 0, 1 );
           if(Trig_Prot & 0xa0)TM_HD44780_PutCustom(6, 1, 0 ); // Imax
           else TM_HD44780_PutCustom(6, 1, 1 );
           TM_HD44780_PutCustom(14, 1, 0 ); break; //вкл-е/выкл защиты - галочки/рамки
  case 2:  TM_HD44780_PutCustom(6, 0, 1 ); TM_HD44780_PutCustom(14, 0, 1 ); TM_HD44780_PutCustom(6, 1, 1 ); TM_HD44780_PutCustom(14, 1, 1 );
        if(Status_Flag & 0x0006)TM_HD44780_PutCustom(6, 0, 0 ); if(Status_Flag & 0x0018)TM_HD44780_PutCustom(14, 0, 0 ); if((Status_Flag & 0x0080)||(Status_Flag & 0x0100))TM_HD44780_PutCustom(6, 1, 0 ); if(Otkaz_IKZ)TM_HD44780_PutCustom(14, 1, 0 ); break; //сраб-е защиты - галочки
  case 3:  IO_Displey (Input_X1); TM_HD44780_Puts(7, 0, IO_Menu ); TM_HD44780_Puts(7, 1, Output_X1 ); break; // вх/вых Х1
  case 4:  IO_Displey (Input_X2 ); TM_HD44780_Puts(7, 0, IO_Menu ); TM_HD44780_Puts(7, 1, Output_X4 ); break; // вх/вых Х4 >>8
  case 5:  if(f_VM_Con==0) TM_HD44780_PutCustom(14, 0, 0 ); else TM_HD44780_PutCustom(14, 0, 1 ); break; //измеритель напр. подключен - галочка 
  case 6:  Var_Displey (Count_Switch); TM_HD44780_Puts(10, 1, Var_Menu ); break; //кол-во вкл/выкл БВ
  case 7:  Var_Displey (LTD_Threshold); TM_HD44780_Puts(7, 1, Var_Menu );break; //настройка ИКЗ
  case 8:  Var_Displey (I1_N); TM_HD44780_Puts(8, 0, Var_Menu ); Var_Displey ((I2_N/10)); TM_HD44780_Puts(8, 1, Var_Menu ); break; // токи LEM-датчиков
  case 9:  Var_Displey (V1_N); TM_HD44780_Puts(8, 0, Var_Menu ); Var_Displey (V2_N); TM_HD44780_Puts(8, 1, Var_Menu ); break; //напряжение 1,2-й канал
  case 10: Var_Displey ((((int16_t)V3_N)/8)); TM_HD44780_Puts(8, 0, Var_Menu );  break; //напряжение 3-й канал
  case 11: Var_Displey_2 (P_Io_Set); TM_HD44780_Puts(6, 0, Var_Menu ); Var_Displey_2 (N_Io_Set); TM_HD44780_Puts(6, 1, Var_Menu ); break; //Ток отсечки
  case 12: Var_Displey_2 (P_Imax_Set); TM_HD44780_Puts(7, 0, Var_Menu ); LCD_Tset[0]=(P_Imax_TSet/10)|48; LCD_Tset[1]=(P_Imax_TSet%10)|48; TM_HD44780_Puts(10, 1, LCD_Tset ); break; //Ток +макс.
  case 13: Var_Displey (N_Imax_Set); TM_HD44780_Puts(7, 0, Var_Menu ); LCD_Tset[0]=(N_Imax_TSet/10)|48; LCD_Tset[1]=(N_Imax_TSet%10)|48; TM_HD44780_Puts(10, 1, LCD_Tset ); break; //Ток +макс.
  case 14: //Var_Displey (P_dIdt_Set/10); TM_HD44780_Puts(6, 0, Var_Menu ); LCD_Tset[0]=(P_dIdt_TSet/10)|48; LCD_Tset[1]=(P_dIdt_TSet%10)|48; TM_HD44780_Puts(10, 1, LCD_Tset ); break; //Ток +dI/dt
  case 15: //Var_Displey (N_dIdt_Set/10); TM_HD44780_Puts(6, 0, Var_Menu ); LCD_Tset[0]=(N_dIdt_TSet/10)|48; LCD_Tset[1]=(N_dIdt_TSet%10)|48; TM_HD44780_Puts(10, 1, LCD_Tset ); break; //Ток -dI/dt
  case 16: Var_Displey (LTD_Izmer/10); TM_HD44780_Puts(7, 1, Var_Menu ); break;
  case 17: Var_Disp8 (Io_TA1); TM_HD44780_Puts(8, 0, Var_Menu ); Var_Disp8 (Io_TA2); TM_HD44780_Puts(8, 1, Var_Menu ); break; //кор. токи LEM-датчиков    if ((int8_t)Io_TA1 > 0){Corr_set [0]=0x2b;} else {}; TM_HD44780_Puts(8, 0, Corr_set ); Var_Displey (Io_TA2); TM_HD44780_Puts(8, 1, Corr_set );
  case 18: Var_Displey (K_V1);   TM_HD44780_Puts(8, 0, Var_Menu ); Var_Displey (K_V2);   TM_HD44780_Puts(8, 1, Var_Menu ); break; //кор. напряжение 1,2-й канал
  case 19: //Var_Displey (K_V3);   TM_HD44780_Puts(8, 0, Var_Menu );  break;  //кор. напряжение 3-й канал
  case 20: if(Trig_Prot & 0x0020)TM_HD44780_PutCustom(14, 1, 0 ); else TM_HD44780_PutCustom(14, 1, 1 ); if(Trig_Prot & 0x0080)TM_HD44780_PutCustom(14, 0, 0 ); else TM_HD44780_PutCustom(14, 0, 1 );  break;  //включ. защиты Imax
  case 21: if(STATUS==0)TM_HD44780_Puts(12, 0, "L" );  if(STATUS==1)TM_HD44780_Puts(12, 0, "I" );  if(STATUS==2)TM_HD44780_Puts(12, 0, "R" ); 
           if(STATUS==3)TM_HD44780_Puts(12, 0, "NL" ); if(STATUS==4)TM_HD44780_Puts(12, 0, "NI" ); if(STATUS==5)TM_HD44780_Puts(12, 0, "NR" ); break;  //изменени режима работы
  case 22: if(Trig_Prot & 0x0002)TM_HD44780_PutCustom(14, 0, 0 ); else TM_HD44780_PutCustom(14, 0, 1 );  if(Trig_Prot & 0x0004)TM_HD44780_PutCustom(14, 1, 0 ); else TM_HD44780_PutCustom(14, 1, 1 ); break;  //вкл/выкл защиты Io+
  case 23: if(Trig_Prot & 0x0008)TM_HD44780_PutCustom(14, 0, 0 ); else TM_HD44780_PutCustom(14, 0, 1 );  if(Trig_Prot & 0x0010)TM_HD44780_PutCustom(14, 1, 0 ); else TM_HD44780_PutCustom(14, 1, 1 );break;  //вкл/выкл защиты dI/dt
  case 24: if(Trig_Prot&0x0001)TM_HD44780_PutCustom(14, 1, 0 ); else TM_HD44780_PutCustom(14, 1, 1 );  break;  //вкл/выкл датчика давления*/
			
  }
  
  /*if(Ch_Knob_Set==2)TM_HD44780_CursorSet(7, 1); //редактируемая строка 2
  else TM_HD44780_CursorSet(7, 0);   //редактируемая строка 1

  if((Ch_Knob_Set!=0)&&((Channel_Set==11)||(Channel_Set==14))) //29_09_20 //(Channel_Set==10)||||(Channel_Set==12)||(Channel_Set==13)
  {
    TM_HD44780_CursorSet(14, 1); 
  }
  
  if((Ch_Knob_Set!=0)&&((Channel_Set==10)||(Channel_Set==12)||(Channel_Set==13))) //05_01_21 //
  {
    if(Ch_Knob_Set==2)TM_HD44780_CursorSet(14, 1); 
    else TM_HD44780_CursorSet(14, 0); 
  }*/
  //мигание курсора-полоска
  if(Ch_Knob_Set == 1) { 
	if(Migan)
  		 ssd1306_Line(68, 15, 84, 15, White); //F_Save =1;
	else ssd1306_Line(68, 15, 84, 15, Black);
	Migan ^= 1;
  } 
  if(Ch_Knob_Set == 2) { 
	if(Migan)
  		 ssd1306_Line(68, 32, 84, 32, White); //F_Save =1;
	else ssd1306_Line(68, 32, 84, 32, Black);
	Migan ^= 1;
  }
  //else TM_HD44780_BlinkOff( );
  
}

uint8_t F_Oled = 0;

void LCD_Podsvetka(void)
{
  if( T_LCD_E != 0 ){ //|| (f_Test==0)
  	T_LCD_E--; if(F_Oled == 0) ssd1306_SetDisplayOn(1); F_Oled = 1;}
  else { ssd1306_SetDisplayOn(0); F_Oled = 0;}
    //GPIOE->BSRR = GPIO_BSRR_BS0;//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
}