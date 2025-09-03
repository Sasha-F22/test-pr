/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "DI_DO.h"
#include "UART1.h"
#include "ADC.h"
#include "LP.h"
#include "OLED.h"
//#include "Tiristor.h"
#include "Algoritm.h"
#include "K_QF1.h"
#include "Control.h"
#include "EEPROM.h"
#include "temp.h"
#include "UART6.h"
#include "Modbus.h"
#include "mb.h"
#include "mbport.h"
#include "port.h"
//#include "tm_stm32_hd44780.h" 


extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;
extern IWDG_HandleTypeDef hiwdg1;  	//temp  02_09_24
extern UART_HandleTypeDef huart_m;
extern HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart);

uint16_t SPI2_Priem = 0;
uint8_t SPI1_testRX = 0;
extern uint8_t TxDO[ ];
extern uint8_t RxDO[ ];
extern uint16_t AnalizSPI[ ];
uint8_t Nomer = 0;
uint8_t Analizer[100] = {0};
uint8_t ch_RX = 0;
uint8_t RxXferCount = 0, PriemN = 0;
uint8_t ch_in = 0;
uint8_t n_T3 = 0;
uint32_t Kali_T2 = 0;

uint8_t  f_TIM2_ = 0;
uint16_t Ch_Start_Sbr = 0; 
uint16_t Ch_10Sek = 0;

uint8_t  T_500mks = 0;
uint32_t isrflags = 0;

uint8_t  Ch_Tir8 = 0;
//__IO uint16_t SPI2_Priem;


//uint8_t aShowTime[50] = {0};
//uint8_t Rus_Modul[]={0x20,0x4D,0x6F,0xE3,0x79,0xBB,0xC4,0x20};
//uint8_t ADC_result [15]={0};

//RTC_TimeTypeDef stimestructureget;
//RTC_DateTypeDef sdatestructureget;
//void RTC_CalendarShow(uint8_t *showtime);

//// UART1 ////
extern uint8_t  RxSize; //№ пакета данных uart1 (ИН)//Счёт принятых байтов UART1 - от изм-ля напр.
extern uint16_t Ch_UART1; //Счёт непринятых пакетов UART1 - от изм-ля напр.
extern uint8_t  f_UART1; //Флаг ошибка непринятых пакетов UART1 - от изм-ля напр.
extern uint8_t  f_CRC_UART1; //Флаг ошибка проверки CRC UART1 - измеритель напр.
extern uint8_t  f_Nach_Pac;
extern uint16_t aRxBuffer[16];//буфер приема данных от Измерителя напряжения
extern uint8_t  V1L,V1H,V2L,V2H,V3L,V3H,Crc_;
uint16_t chemaz = 0;

//// UART8 ////
extern uint16_t Ch_UART8; //Счёт непринятых пакетов UART1 - от изм-ля напр.
extern uint8_t  f_UART8;

extern uint8_t f_SPI_CRC;  		//Флаг ошибка проверки CRC SPI1 - дискретные вых.
extern uint8_t f_SPI_OpenL;		//для проверки
extern uint8_t f_SPI_UVLO;		//для проверки
extern uint8_t f_SPI_Err;		//для проверки
extern uint8_t Ch_SPI1;
extern uint8_t Ch_SPI2;
extern uint8_t f_SPI1_;
extern uint8_t f_SPI2_;
//extern uint8_t Ch_Nach_P;

uint8_t  f_Zapusk = 1; 			//флаг обнуления флагов при запуске
//uint16_t Ch_10Sek = 0;
uint8_t  Fault_04 = 0;
extern uint8_t  Fault_18;
extern uint8_t  f_Low_24V;
extern uint8_t  F_I2C_Wr;
extern uint8_t  F_I2C_Rd; 
uint16_t Ch_I2C = 0; 			//Счетчик от зависания 
//extern uint8_t  F2_I2C_Wr;
//extern uint8_t  F2_I2C_Rd;

#define inp1_1 ((Input_X1 & 0x0001)>>0) //QS1 отключен
#define inp1_2 ((Input_X1 & 0x0002)>>1) //QS1 включен
#define inp1_4 ((Input_X1 & 0x0008)>>3)
#define inp1_5 ((Input_X1 & 0x0010)>>4)
#define inp1_6 ((Input_X1 & 0x0020)>>5)

extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
extern  uint8_t  Gash_Duge;
extern  uint8_t  Faultec ;
extern  uint32_t FAULT ;
extern  uint8_t  F_Ch_Otkl;

uint8_t Ch_SMTN = 0;
uint8_t Ch_Otkl = 0;
uint8_t f_VM_Con = 0;

//uint8_t f_NSS1 = 0;
uint8_t f_F10_Start = 1; 	//Флаг для стирания ошибки 10 при подаче питания
uint16_t Ch_F10_St = 0; 	//Счетчик для стирания ошибки 10 при подаче питания


extern uint8_t f_SSh;  		//08_06_21
extern uint8_t F_AFB_Good;
extern uint8_t IGCT_Test;
extern uint8_t F_Cap_Contr;
extern uint8_t TxDO[6];
extern uint8_t F_Pisk;
uint8_t ch_Pisk = 0;
uint8_t Pisk_ = 0;
uint8_t f_faza = 0;
//////debag/////
uint16_t TestIme = 0;
int16_t TestBuffer[16];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c4;
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Tamper and TimeStamp interrupts through the EXTI line.
  */
void TAMP_STAMP_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_STAMP_IRQn 0 */

  /* USER CODE END TAMP_STAMP_IRQn 0 */
  HAL_RTCEx_TamperTimeStampIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_STAMP_IRQn 1 */

  /* USER CODE END TAMP_STAMP_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  //HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
	//DMA1_Stream0->CR &= ~DMA_SxCR_TCIE;
  	DMA1->LIFCR = DMA_LIFCR_CFEIF0 |DMA_LIFCR_CDMEIF0 |DMA_LIFCR_CTEIF0 
    |DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 ;// Сброс флага прерывания потока 0 DMA2
	
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    SPI1->CFG1|= SPI_CFG1_TXDMAEN;
	
	//f_NSS1=0;
    Ch_SPI1=0;
	
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  //HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
  
	//DMA1_Stream1->CR &= ~DMA_SxCR_TCIE;
	NVIC_DisableIRQ(DMA1_Stream1_IRQn);

	DMA1->LIFCR = DMA_LIFCR_CFEIF1 |DMA_LIFCR_CDMEIF1 |DMA_LIFCR_CTEIF1 |DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  ///* Таймер на 50 mks*/
  //закоментить стандартный обработчик  ниже
  /* USER CODE END TIM2_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  
    /* TIM Update event */
  if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) //проверка и очистка флага прерывания
  {
	if(__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) !=RESET)
	{
	  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	}
	else
	  GPIOA->BSRR = GPIO_BSRR_BS4;  //test
  
  
  /////// наброски нового кода под AFB15k ///////
  TestBuffer[0] = 0; TestBuffer[1] = 0; TestBuffer[2] = 0; TestBuffer[3] = 0; TestBuffer[4] = 0; TestBuffer[5] = 0; TestBuffer[6] = 0; TestBuffer[7] = 0;
  if( f_faza )  //100мксек
  {	  
	//USART1->CR1 &= 0xfffffff8; //15K ? запрет приема USART1
	UART8->CR1 |= 0x00000005; //2-й канал
	
	GPIOA->BSRR = GPIO_BSRR_BS4; //NSS SPI1 -V
	__HAL_SPI_DISABLE(&hspi1);  __HAL_SPI_DISABLE(&hspi2);
	uint16_t XX = 0;
	XX =(uint16_t)(READ_REG (SPI1->RXDR)); XX =(uint16_t)(READ_REG (SPI1->RXDR));
	  
	GPIOA->BSRR = GPIO_BSRR_BR4; //NSS SPI1
	SPI_1_CRC();
	ch_in = 0; ch_RX = 0; Nomer = 0; 
	RxXferCount = 5; PriemN = 0; n_T3 = 0; 
	AnalizSPI[10] = DWT->CYCCNT; 
	SPI_1_Rx_Tx();
	SPI_1_Diagnostic();
	///мигание светодиода, тест 
	//if(Kali_T2 < 200000) Kali_T2 ++;
	//else {Kali_T2 = 0; TxDO[2] ^= 1;}

	if (Gash_Duge == 0)//импульсы гашения отсутствуют
	{    
		SPI_2_Rx_Tx();
	  
		if (F_I2C_Wr == 1) Zapis_EEPROM(0x01); 	//&& (F_I2C_Rd==0)test
		if ((F_I2C_Wr == 2) ) Zapis_EEPROM(0x02); 	//&& (F_I2C_Rd==0)test
		if ((F_I2C_Rd == 1) && (F_I2C_Wr == 0)) Chtenie_EEPROM(0x01); 	//test
		if ((F_I2C_Rd == 2) && (F_I2C_Wr == 0)) Chtenie_EEPROM(0x02); 	//test
		if ((F_I2C_Wr != 0) && (F_I2C_Rd != 0)) FAULT |= 0x80000000; else  FAULT &= ~0x80000000;	//test
		
		Contr_Deb();
	  
		SPI_2_Diagnostic();
	}
	
	Ch_UART8 ++; //счёт не принятых пакетов по UART8(в том числе с неправильным CRC)
	if (Ch_UART8 > 0x1f) //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // было 0x01f
	{
	  f_UART8 = 1;//f_CRC_UART1 = 0;
	  f_VM_Con = 1;
	}
	//else f_VM_Con = 0;
  }
  else 
  {
	USART1->CR1 |= 0x00000005; //15K ? разреш-е приема USART1
	//UART8->CR1 &= 0xfffffff8; //2-й канал
	
	Ch_UART1 ++; //счёт не принятых пакетов по UART1(в том числе с неправильным CRC)
	if (Ch_UART1 > 0x0f) //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // было 0x01f
	{
	  f_UART1 = 1; 
	  f_CRC_UART1 = 0;
	  f_VM_Con = 1;
	}
	//else f_VM_Con = 0; 
	
  }
  f_faza ^= 1;
  
  f_TIM2_ = 1;
  
  f_VM_Con = f_UART1 || f_UART8;
  //__HAL_IWDG_RELOAD_COUNTER(&hiwdg); //HAL_IWDG_Refresh(&hiwdg);
    

  /////////////////////////////////////////////////////////////////////////////
  if (f_Zapusk == 0) //задержка при включение, подаче питания
  {
	Fault_04 |= (f_SPI1_||f_SPI2_ ||f_UART1 || f_UART8); 	//
	f_UART1 = 0; f_UART8 = 0;
	ADC_Start();
	//GPIOE->BSRR = GPIO_BSRR_BR1;				//6-7    0.4us/7,1 /5
	ADC_Norm();
	if (Gash_Duge == 0) Proverka_Zashit(); 		//  
	//GPIOE->BSRR = GPIO_BSRR_BS1;				//7-8    0.4us/1  /0.7
	Input_X1 = Input_Optron();
	//GPIOE->BSRR = GPIO_BSRR_BR1;				//8-9    0.9us/1  /0.7
	Input_X2 = In_X4_Deb()>>8;
	//GPIOE->BSRR = GPIO_BSRR_BR1;				//10-11   1.2us/1  /0.7
	Proverka_Pered_Vkl(); 						//if (F8_Proverka)
	//GPIOE->BSRR = GPIO_BSRR_BS1;				//11-12   0.2us/1  /0.7

	// 1/10
	if ( T_500mks ) T_500mks--;
	else 
	{
	  T_500mks = 9;
	  Poschitat_2(); 							//процессы с периодом 500мкс
	  BV_Process();
	  if(F_Pisk){
		if(Pisk_) GPIOF->BSRR = GPIO_BSRR_BR8;
		else GPIOF->BSRR = GPIO_BSRR_BS8;
		Pisk_ ^= 1 ;}
	#ifdef AFB_40 
	  IGCT_Control(); //06_04_19
	#endif
	}

	//GPIOE->BSRR = GPIO_BSRR_BS1;//13-14   1.0
	Algoritm();
	QF1_Control(); //20_03_19
	Poschitat();   	////расчет  значений 21_10_21 поменял порядок
	Tx_SMTN (); //перенос в   100мксек цикл
	//GPIOE->BSRR = GPIO_BSRR_BR1; //6-15 10.4мкс /14-15  5.0    TEMP
  }
  else
  {
  	f_SPI1_= 0; f_SPI2_= 0; f_UART1 = 0;
  }
  
  HAL_IWDG_Refresh(&hiwdg1); //
  
  }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* Таймер на 20/5 mks */
  //Закоментировать строку ниже
  /* USER CODE END TIM3_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  
  if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) //проверка и очистка флага прерывания
  {
    //if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    }
  
  //делить на 8
	if(Ch_Tir8 > 7) {Imp_tirist ( ); Ch_Tir8 = 0; }
	Ch_Tir8 ++;
  
  //if( ((SPI1->SR & SPI_SR_EOT) || f_NSS1) == 0 ) GPIOA->BSRR = GPIO_BSRR_BS4; //SPI1_NSS  SPI_SR_BSY?
	n_T3 ++;
	  //if(ch_in < 100){
		//Analizer[ch_in] = ((GPIOA->IDR) & 0xe0)>>5;ch_in ++;}
	/// Check the RXP flag //
	  if (((SPI1->SR&SPI_FLAG_RXP)!=0) && (RxXferCount > 0UL))
      {
        RxDO[PriemN] = *((__IO uint8_t *)&SPI1->RXDR); PriemN ++;
        RxXferCount --;
		AnalizSPI[PriemN + 10] = DWT->CYCCNT; // TIM2->CNT
      }
	  if (n_T3 >12) {	
		SPI1->CR1 &= ~SPI_CR1_SPE;
		GPIOA->BSRR = GPIO_BSRR_BS4;
	  	//TIM3->DIER =0;
	  }
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  /* Таймер на 10 ms */   //или 1мсек
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  
  //GPIOF->BSRR = GPIO_BSRR_BS12; //TEMP
  // LCD_Podsvetka(); -> OLED
  Key_Status();
  
#ifdef AFB_40 
    //if((F_AFB_Good) && (IGCT_Test==2)) f_SSh =1; //09_06_21 - команда развести разъед при подаче питания 
   if((F_Cap_Contr) && (IGCT_Test==2)) f_SSh =1; //12_10_21
   IGCT_Contr (); //перенос 27_04_22
#endif
  
  if (Ch_Start_Sbr == 152) //
  { 
    //if ((inp1_1 ==0)||(inp1_2 ==1)) f_SSh =1;   //Input_X4 |= 0x0004;F8_QS1 =2;  
    if (inp1_4 == 1) {Fault_18 = 1; Faultec = 1;} //если после включения БВ вкл-н  29_03 (Fault_14 -> 18)
    if (inp1_5 || inp1_6){ FAULT |= 0x80; } 	// 11_11_19 - проверка контакторов при включение
    Ch_Start_Sbr = 0;
  } 
  
  Ch_Start_Sbr += (f_Zapusk && (f_Low_24V == 0));
  
  if((Ch_Start_Sbr > 150) && f_Zapusk) 		//задержка при включение 100*10ms = 1s (после f_Low_24V==0)
  {
    Ch_Start_Sbr ++;
    f_UART1 = 0; 
    f_Zapusk = 0;
    F_I2C_Rd = 2; //21_03_24
  } 
  
  Ch_10Sek++; //500мс вместо 10сек
  if (Ch_10Sek > 50) {
    usRegHoldingBuf[46] = (((TxDO[0]&1)<<7)|((TxDO[0]&2)<<5)|((TxDO[0]&4)<<3)  //было "usRegHoldingBuf[43]"
						   |((TxDO[0]&8)<<1)|((TxDO[0]&16)>>1)|((TxDO[0]&32)>>3)
							 |((TxDO[0]&64)>>5)|((TxDO[0]&128)>>7));
    Status_Flag ^= 0x8000; Ch_10Sek = 0; 		//0/1через каждые 0,5 сек для проверки связи
  } 
  
   Menu_(); //OLED
  
  if(f_Zapusk == 0){
    if (Ch_F10_St > 199) QS1_Control();
    KM1_Control();
    KM2_Control();
    Fan_Control();
    Cap_Control(); 
    //IKZ_Control();
    Rozriad_C12(); 						//20_01_20
	Sled(); 							//19_03_25
  }
  
  //Обнуление 10-й ошибки при включение - после дорозвода разъединителя
  if ((f_F10_Start ) && (f_Zapusk == 0))
  {
    if (Ch_F10_St > 899) {f_F10_Start = 0; FAULT &= 0xfffffdff;}
    Ch_F10_St ++;
  }
  
  //запись в EEPROM
  if(F_Ch_Otkl) //задержка времени на запись в ППЗУ //100мсек
  {
    Ch_Otkl ++; 
    if((Ch_Otkl == 10) && (F_Ch_Otkl == 1)) {F_I2C_Wr = 1; F_Ch_Otkl = 0;}
    //if((Ch_Otkl==10)&&(F_Ch_Otkl==2)){F2_I2C_Wr=1; F_Ch_Otkl=0;}//03_09 
  }
  else Ch_Otkl = 0;
  
  //Pisk
  if(F_Pisk ) {
	if(ch_Pisk < 14) ch_Pisk ++;
	else {ch_Pisk = 0; F_Pisk = 0; GPIOF->BSRR = GPIO_BSRR_BR8;}
	//GPIOF->BSRR = GPIO_BSRR_BR8;
  }
  
  
  //GPIOF->BSRR = GPIO_BSRR_BR12; //TEMP 3-5мкс  

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
//Выходы
  
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */
  //Входы
	// закоментировать стандартный обработчик прер-я
  /* USER CODE END SPI2_IRQn 0 */
  //HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */
  
  /* Check the RXNE flag */
  if((hspi2.Instance->SR)&0x01)//__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE)
  {        
      uint16_t temp = (SPI2 -> RXDR);
      if (temp != 0){
		SPI2_Priem = temp; Ch_SPI2 = 0;
	  }
      GPIOB->BSRR = GPIO_BSRR_BS12; //GPIOB->BSRR = GPIO_BSRR_BS15;
      Ch_SPI2 = 0; CLEAR_BIT(SPI2->CR1, SPI_CR1_CSTART);
      __HAL_SPI_DISABLE(&hspi2);
      __HAL_UNLOCK(&hspi2);
  }

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

   //GPIOF->BSRR = GPIO_BSRR_BS12; // TEMP // TEMP 
   isrflags = USART1->ISR;
   chemaz = (USART1->RDR) & 0x01FF;
   //GPIOF->BSRR = GPIO_BSRR_BR12; //0.4мкс TEMP // TEMP 
   if (isrflags & 0x20) //
   {
      if (chemaz == 0x0155) RxSize = 0; 				//признак начала пакета
      else
      {
        RxSize ++;
        if(RxSize > 14) {f_Nach_Pac = 1; RxSize = 0;}	//флаг ошибки
        aRxBuffer[RxSize] = chemaz;						//сохранение в буфер данных
        if (RxSize == 7) 								//пакет данных получен
        {
          V1L = aRxBuffer[1];
          V1H = aRxBuffer[2];
          V2L = aRxBuffer[3];
          V2H = aRxBuffer[4];
          V3L = aRxBuffer[5];
          V3H = aRxBuffer[6];
          Crc_= aRxBuffer[7];
		  USART1->CR1 &= 0xfffffff8;
          //GPIOF->BSRR = GPIO_BSRR_BS12; // TEMP // TEMP   
          CRC_Controll (1); //~2.5мкс / стало 1,5мкс
          //GPIOF->BSRR = GPIO_BSRR_BR12; //мкс TEMP // TEMP 
        }
      }
     // Rx_UART ( chemaz, &huart1 ); //
   }
   
   USART1->ICR = 0xffffff; //18_04_25 очистка прерываний флагов
   
   
   isrflags = USART1->ISR;
  //закоментить стандартный обработчик прерывания
  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
//void USART2_IRQHandler(void)
//{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  //HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
//}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	ADC_Results();
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */
	uint32_t  tickstart = TIM2->CNT; // TEMP // TEMP // DWT->CYCCNT;
	isrflags = UART8->ISR;
   chemaz = (UART8->RDR) & 0x01FF;
   //GPIOF->BSRR = GPIO_BSRR_BR12; //0.4мкс TEMP // TEMP 
   if (isrflags & 0x20) //
   {
      if (chemaz == 0x0155) 
		RxSize = 0; 				//признак начала пакета
      else {
        RxSize ++;
        if(RxSize > 14) {f_Nach_Pac = 1; RxSize = 0;}	//флаг ошибки
        aRxBuffer[RxSize] = chemaz;						//сохранение в буфер данных
        if (RxSize == 7) 								//пакет данных получен
        {
          V1L = aRxBuffer[1];
          V1H = aRxBuffer[2];
          V2L = aRxBuffer[3];
          V2H = aRxBuffer[4];
          V3L = aRxBuffer[5];
          V3H = aRxBuffer[6];
          Crc_= aRxBuffer[7];
		  UART8->CR1 &= 0xfffffff8;
          //GPIOF->BSRR = GPIO_BSRR_BS12; // TEMP // TEMP   
          CRC_Controll (2); //~2.5мкс / стало 1,5мкс
          //GPIOF->BSRR = GPIO_BSRR_BR12; //мкс TEMP // TEMP 
        }
      }
   }
   
   UART8->ICR = 0xffffff; //18_04_25 очистка прерываний флагов
   TestBuffer[RxSize] = (TIM2->CNT) - tickstart;
   
   isrflags = UART8->ISR;
  /* USER CODE END UART8_IRQn 0 */
  //HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/**
  * @brief This function handles SPI6 global interrupt.
  */
void SPI6_IRQHandler(void)
{
  /* USER CODE BEGIN SPI6_IRQn 0 */

  /* USER CODE END SPI6_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi6);
  /* USER CODE BEGIN SPI6_IRQn 1 */

  /* USER CODE END SPI6_IRQn 1 */
}

/**
  * @brief This function handles I2C4 event interrupt.
  */
void I2C4_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C4_EV_IRQn 0 */

  /* USER CODE END I2C4_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c4);
  /* USER CODE BEGIN I2C4_EV_IRQn 1 */

  /* USER CODE END I2C4_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void IAR_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t tmp_flag = 0, tmp_it_source = 0;

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    prvvUARTRxISR(  ); 
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    prvvUARTTxReadyISR(  );
  } 
}

void USART2_IRQHandler(void)
{
  IAR_UART_IRQHandler(&huart_m);
}
/* USER CODE END 1 */
