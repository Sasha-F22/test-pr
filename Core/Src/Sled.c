
/* Includes ------------------------------------------------------------------*/
#include "SDCard.h"
//#include "stm32h7xx_it.h"
//#include "stm32h735xx.h"
#include "stm32h7xx_hal.h"
#include "Modbus.h"
/* Declarations and definitions ----------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
uint8_t Buff_Sled [512] = {0}; // 1ms?
//uint8_t TA2_Sled [16] = {0};

uint8_t  Sled_Wr = 1; //вкл. реж. записи следа
uint32_t blockNum = 3;
uint8_t  iSled = 0;
extern SPI_HandleTypeDef SDCARD_SPI_PORT;
uint8_t  f_Avar = 0;
static uint8_t n_Avar = 0;

/* Functions -----------------------------------------------------------------*/

void Sled(void) 	//1ms 
{
  if( Sled_Wr == 1){  // Sled_Wr = 0 после отключения, знач-е iSled во время "аварии" запомнить Avar = iSled;
	if(iSled < 16){
	  iSled ++;
	}
	else iSled = 0;
	
	Buff_Sled [iSled + 0 ] = I1_N_1_6ms >> 8;
	Buff_Sled [iSled + 16] = I2_N_1_6ms >> 8;
	Buff_Sled [iSled + 32] = V1_N_ >> 8;
	Buff_Sled [iSled + 48] = V2_N_ >> 8;
	
	
	if(f_Avar){
		if(n_Avar < 8) n_Avar ++; 
		else {Sled_Wr = 2; f_Avar = 0; n_Avar = 0;}} //
	
  }
  
  if( Sled_Wr == 2) 
  { 
	//Сохр-е следа на SDCard
	//blockNum = номер пред. запмсанного блока +1, если n > Nmax то N = 3
	//SDCARD_GetBlocksNumber(uint32_t* num);
	//SDCARD_ReadSingleBlock(2, Buff_SD); //номер записи
	SDCARD_WriteSingleBlock(blockNum, Buff_Sled);
	Sled_Wr = 3;
  }
    //HAL_GPIO_WritePin(SDCARD_CS_GPIO_Port, SDCARD_CS_Pin, GPIO_PIN_RESET);
}



/*----------------------------------------------------------------------------*/






//////// The end ///////