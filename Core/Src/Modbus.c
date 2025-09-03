
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "Modbus.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 30001
#define REG_INPUT_NREGS 255
#define REG_HOLDING_START 40001
#define REG_HOLDING_NREGS 255 
/* ----------------------- Static variables ---------------------------------*/
extern  USHORT   usRegInputStart ; //регистры ввода
extern  USHORT   usRegInputBuf[REG_INPUT_NREGS];
extern  USHORT   usRegHoldingStart ; //регистры хранения 
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //

/* ------ variables ---------------------------------------------------------*/

/*struct  usRegHoldingBuf { Status_Flag, 0, 0, FAULT_Modbus1, FAULT_Modbus2, 0,0,
 0,0, 0xffff, 0,0,Count_Switch,CT1_QS1,0,0,0,0,0, P_Io_Set, N_Io_Set, 0, P_Imax_Set, 
 P_Imax_TSet, N_Imax_Set, N_Imax_TSet, 0, P_dIdt_Set, P_dIdt_TSet, N_dIdt_Set, 
 N_dIdt_TSet, 0, LTD_Threshold, 0,0,I1_Abs0, 0,0,0, (USHORT)I1_N_1_6ms, (USHORT)dI_dt_1_6ms, 
 0, Input_X1, TxDO[0], Input_X4, TxDO[3],0,0,0,0,Status_Bit};*/



//uint16_t Status_Flag =0; //r0
/*extern uint16_t FAULT_Modbus1;//r3
extern uint16_t FAULT_Modbus2;//r4
extern uint16_t Count_Switch;//r12
extern uint16_t CT1_QS1; 
extern uint16_t P_Io_Set;
extern uint16_t N_Io_Set;
extern uint16_t P_Imax_Set;
extern uint16_t N_Imax_Set;
extern uint16_t P_Imax_TSet;
extern uint16_t N_Imax_TSet;
extern uint16_t P_dIdt_Set;
extern uint16_t N_dIdt_Set;
extern uint16_t P_dIdt_TSet;
extern uint16_t N_dIdt_TSet;
extern uint16_t LTD_Threshold;
extern int I1_Abs0;
//extern int16_t I1_N_1_6ms; // 1.6мс или 32 измерений через 0,05мс каждое 
//extern int16_t dI_dt_1_6ms;uint8_t FAULT;
extern uint16_t Input_X1;
extern uint16_t Input_X4;
extern uint8_t TxDO[6];

extern volatile int16_t I1_N;
 // значение АЦП полученное при выключенном БВ - инвертированное (добавить)*/

/*int16_t I1_N_64ms=0;// 64мс или 128 измерений через 0,5мс каждое
int16_t dI_dt_64ms=0;

int16_t I_SMTN=0;
uint8_t Status_Bit=0;*/


/* ------- functions ---------------------------------------------------------*/

/*void Modbus_Reg_Wr(void) //100 ms 
{
  usRegHoldingBuf[0]=Status_Flag;
  usRegHoldingBuf[3]=FAULT_Modbus1;
  usRegHoldingBuf[4]=FAULT_Modbus2;
  usRegHoldingBuf[9]=0xffff;
  usRegHoldingBuf[12]=Count_Switch; //СТ1_QF1
  usRegHoldingBuf[13]=CT1_QS1; 
  usRegHoldingBuf[19]=P_Io_Set;
  usRegHoldingBuf[20]=N_Io_Set;
  usRegHoldingBuf[22]=P_Imax_Set;
  usRegHoldingBuf[23]=P_Imax_TSet;
  usRegHoldingBuf[24]=N_Imax_Set;
  usRegHoldingBuf[25]=N_Imax_TSet;
  usRegHoldingBuf[27]=P_dIdt_Set;
  usRegHoldingBuf[28]=P_dIdt_TSet;
  usRegHoldingBuf[29]=N_dIdt_Set;
  usRegHoldingBuf[30]=N_dIdt_TSet;
  usRegHoldingBuf[32]=LTD_Threshold;
  usRegHoldingBuf[35]=I1_Abs0;
  //usRegHoldingBuf[39] = 100;//(USHORT)I1_N_1_6ms;
  //usRegHoldingBuf[40] = 144;//(USHORT)dI_dt_1_6ms;
  usRegHoldingBuf[42]=Input_X1;
  usRegHoldingBuf[43]=TxDO[0];
  usRegHoldingBuf[44]=Input_X4;
  usRegHoldingBuf[45]=TxDO[3];
}


void Modbus_Reg_Rd(void) //запрос по Модбасу
{
  Status_Bit = usRegHoldingBuf[50];
  if(usRegHoldingBuf[51])P_Io_Set = usRegHoldingBuf[51];
  if(usRegHoldingBuf[52])N_Io_Set = usRegHoldingBuf[52];
  if(usRegHoldingBuf[53])P_Imax_Set = usRegHoldingBuf[53];
  if(usRegHoldingBuf[54])P_Imax_TSet = usRegHoldingBuf[54];
  if(usRegHoldingBuf[55])N_Imax_Set = usRegHoldingBuf[55];
  if(usRegHoldingBuf[56])N_Imax_TSet = usRegHoldingBuf[56];
  if(usRegHoldingBuf[57])P_dIdt_Set = usRegHoldingBuf[57];
  if(usRegHoldingBuf[58])P_dIdt_TSet = usRegHoldingBuf[58];
  if(usRegHoldingBuf[59])N_dIdt_Set = usRegHoldingBuf[59];
  if(usRegHoldingBuf[60])N_dIdt_TSet = usRegHoldingBuf[60];
  if(usRegHoldingBuf[61])LTD_Threshold = usRegHoldingBuf[61];
  I_SMTN = usRegHoldingBuf[63];
  if (Status_Bit&0x0002){FAULT_Modbus1 =0; FAULT_Modbus2 =0; Status_Flag=0;}
}*/

