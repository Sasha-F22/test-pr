/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/


//#include "mb.h"
#include "mbport.h"
#include "port.h"
#include "Modbus.h"

#include "DI_DO.h"
#include "ssd1306.h"
#include "ADC.h"
#include "main1.h"
#include "SDCard.h"

/

/* Private variables ---------------------------------------------------------*/


 int16_t n2 = 0;
 uint8_t state = 0; //25_03_24


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///инструменты отладки
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Разрешаем DWT
    DWT->CYCCNT = 0;                                // Сброс счётчика
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Запуск счётчика
}

//////////////////////////////////// критическая секция из примера по Modbus 
//static uint32_t primask = 0;
void __critical_enter(void)
{   
  //primask = __get_PRIMASK(); //???
  //__disable_irq();
}

void __critical_exit(void)// Unlock interrupts only when we are exiting the outermost nested call. 
{   
  //if (!primask) __enable_irq(); //???
}


//...........................................................................//
// задать регистры для Модбас стека
/* ----------------------- Defines modbus -----------------------------------*/
#define REG_INPUT_START 30001
#define REG_INPUT_NREGS 255
#define REG_HOLDING_START 40001
#define REG_HOLDING_NREGS 255
/* ----------------------- no-Static variables ------------------------------*/
 USHORT   usRegInputStart = REG_INPUT_START; //static регистры ввода
 USHORT   usRegInputBuf[REG_INPUT_NREGS]; //static
 USHORT   usRegHoldingStart = REG_HOLDING_START; //staticрегистры хранения 
 USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //static
 
/* ----------------------- Defines ------------------------------------------*/
//#define AFB_40 // для 4-ки, комментируем для 25
#define BKPSRAM_BASE 30001
 
uint8_t  Type_AFB = 15; 

uint16_t Nomer_AFB = 6;
uint16_t PO_Ver_AFB = AFB_PO_Ver;  //изменение версии ПО перенос в хедер

extern uint16_t  Ch_IGCT ;
//extern uint8_t STATUS;
	char TextSt[] = "AFB_2000 ";

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	
  while (1)
  {

	nop;	
  }
  /* USER CODE END 3 */
}

