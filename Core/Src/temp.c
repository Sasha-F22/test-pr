
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Modbus.h"
#include "temp.h"

/* ------ variables ---------------------------------------------------------*/
extern volatile int16_t I1_N;
extern volatile int16_t I2_N; //27_08_20

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 30001
#define REG_INPUT_NREGS 255
#define REG_HOLDING_START 40001
#define REG_HOLDING_NREGS 255 
/* ----------------------- Static variables ---------------------------------*/
extern  USHORT   usRegInputStart ; //регистры ввода
extern  USHORT   usRegInputBuf[REG_INPUT_NREGS];
extern  USHORT   usRegHoldingStart ; //регистры хранения 
extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //

int16_t I1_N_0=0;
int16_t I1_N_1=0;
int16_t I1_N_2=0;
int16_t dI_dt_50mks=0;
int16_t dI_dt_100mks=0;
int16_t dI_dt_150mks=0;
int16_t I1_N_64ms=0;	// 64мс или 128 измерений через 0,5мс каждое
int16_t dI_dt_64ms=0;
int32_t Filtr_dI=0;
int16_t dI_dt_50mks_F=0;

int16_t I1_N_max=0;
int16_t I1_N_min=0;
//int16_t I1_N_buff1[]={0,0,0,0,0};
int16_t I1_N_buff2[128]={0};
int16_t I1_N_buff3[32]={0};

uint8_t f_Zap_Buff_I1=0;
uint8_t n_filtra=2;

int16_t I2_N_64ms=0;	// 64мс или 128 измерений через 0,5мс каждое  //27_08_20
int16_t I2_N_buff2[128]={0}; //27_08_20

void Poschitat(void) //50 mks //расчет средних значений 50мкс
{
	dI_dt_150mks = (int16_t)I1_N - (int16_t)I1_N_2;
	dI_dt_100mks = (int16_t)I1_N - (int16_t)I1_N_1;
	dI_dt_50mks  = (int16_t)I1_N - (int16_t)I1_N_0;
	I1_N_2 = I1_N_1; I1_N_1 = I1_N_0; I1_N_0 = I1_N;
	//фильтерим
	Filtr_dI += dI_dt_50mks - dI_dt_50mks_F;
	dI_dt_50mks_F = Filtr_dI /n_filtra; //

	///1.6ms
	static uint8_t i_1_6ms;
	static int32_t S_I1_1_6ms;
	dI_dt_1_6ms = (int16_t)(I1_N - I1_N_buff3[i_1_6ms]);//?              //if (dI_dt_1_6ms & 0x8000)S_I1_1_6ms += (uint16_t) dI_dt_1_6ms | 0xffff0000; //dI_dt_1_6ms & 0xffff;
	S_I1_1_6ms += (int16_t) dI_dt_1_6ms ;
	I1_N_buff3[i_1_6ms]=I1_N;
	i_1_6ms++;
	I1_N_1_6ms = (S_I1_1_6ms/10);
	if(i_1_6ms > 9)i_1_6ms=0;

	I1_N_max=(I1_N_max < I1_N) ? I1_N : I1_N_max;
	I1_N_min=(I1_N_min > I1_N) ? I1_N : I1_N_min;
}


void Poschitat_2(void) //500 mks //расчет средних значений 64ms - вызывается раз в 500мкс
{
    ///64ms
	static uint8_t i_64ms;
	static int32_t S_I1_64ms;
	static int32_t S_I2_64ms;//27_08_20
	  
	S_I1_64ms+=I1_N-I1_N_buff2[i_64ms];
	dI_dt_64ms=I1_N-I1_N_buff2[i_64ms];//?
	I1_N_buff2[i_64ms]=I1_N;
	
	S_I2_64ms+=I2_N-I2_N_buff2[i_64ms];//27_08_20
	I2_N_buff2[i_64ms]=I2_N;  //27_08_20
	I2_N_64ms=S_I2_64ms >> 7; //27_08_20
	
	i_64ms++;
	I1_N_64ms=S_I1_64ms >> 7;
	if(i_64ms>127)i_64ms=0;
      
      //usRegHoldingBuf[39] = 100;//(USHORT)I1_N_1_6ms;
      //usRegHoldingBuf[40] = 144;//(USHORT)dI_dt_1_6ms;
      //usRegHoldingBuf[REG_HOLDING_NREGS]; /
}

