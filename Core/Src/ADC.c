
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "ADC.h"
#include "Modbus.h"

/* Private variables ---------------------------------------------------------*/
 int16_t I1_Abs;       // volatile данные АЦП первого канала
 int16_t I2_Abs;       // volatile данные АЦП второго канала
 int16_t I1_N;         //volatileТок 1 в амперах
 int16_t I2_N;         // volatile Ток 2 в амперах
 uint16_t time_s = 0;
 

 uint16_t f_ADC = 0;
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //

void ADC_Init(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);       // CS = 0 всегда ноль
}

 ////////////////////////  формирование сигналов для запуска/чтения АЦП  ////////////////////
    /*  выходы  	          	PG:8 - (RD)считывание показаний, по спаду;
                                PA:8 - (CS)синхронизация, 0 - разрешает ; 
                                PG:6 - (CONVST)запуск преобр-я, по фронту;
								PG:7 - (RST)

      входы                     PA:11 - (BUSY)преобразование идет;
                              	PA:12 - первый канал измерения;
    */
#pragma optimize = none
void ADC_Start(void)
{
  //TEMP - Уменьшил время выполнения с 1 мкс до 100 нс
  if(GPIOA->IDR & GPIO_PIN_11)  //test
	GPIOG->BSRR = GPIO_BSRR_BR6;
  GPIOG->BSRR = GPIO_BSRR_BR6;          // запуск преобр. = 0
  uint32_t start = DWT -> CYCCNT;
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");  __ASM("NOP");
  GPIOG->BSRR = GPIO_BSRR_BS6; // запуск преобр. = 1

  uint32_t fin = DWT -> CYCCNT;
  time_s = fin - start;
}

void ADC_Results(void)//считывание результата после преобразования 
{
  //TEMP - Уменьшил время выполнения с 2,8 мкс до 300 нс
  //1-й канал тока
  GPIOG->BSRR = GPIO_BSRR_BR8;                  // RD = 0                                  
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  I1_Abs = GPIOD->IDR;    
  I1_Abs &= (~0x04); 	// PD2
  I1_Abs |= (GPIOG->IDR & GPIO_PIN_12) >> 10;
  
  GPIOG->BSRR = GPIO_BSRR_BS8;                  // RD = 1
  __ASM("NOP");
  __ASM("NOP"); 
  //2-й канал тока
  GPIOG->BSRR = GPIO_BSRR_BR8;                  // RD = 0
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");			// while (GPIOG->IDR & GPIO_PIN_8){} //ждем
  I2_Abs = GPIOD->IDR;                 
  I2_Abs &= (~0x04); 	//
  I2_Abs |= (GPIOG->IDR & GPIO_PIN_12) >> 10;
  GPIOG->BSRR = GPIO_BSRR_BS8;                  // RD = 1
  //__ASM("NOP");
  //__ASM("NOP");
  /*//3-й канал АЦП
  GPIOG->BSRR = GPIO_BSRR_BR5;                  // RD = 0 
  __ASM("NOP");
  __ASM("NOP");                                                                                              
  GPIOG->BSRR = GPIO_BSRR_BS5;                  // RD = 1
  __ASM("NOP");
  __ASM("NOP");
  //4-й канал АЦП
  GPIOG->BSRR = GPIO_BSRR_BR5;                  // RD = 0 
  __ASM("NOP");
  __ASM("NOP");
  GPIOG->BSRR = GPIO_BSRR_BS5;                  // RD = 1
  __ASM("NOP");
  __ASM("NOP");*/
}

void ADC_Reset(void)//сброс внешнего АЦП после включения
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET); // Сброс АЦП - фронт
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET); // Сброс АЦП - спад 
}

//const uint16_t Koef1=324; //317.5 расчет
uint16_t Koef2 = 60; //300
//__no_init uint8_t I1_Abs0 @0x20000040; // значение АЦП полученное при выключенном БВ - инвертированное (добавить)

void ADC_Norm(void)//привод к действ значениям тока
{
  //I1_Abs0=5;
  //I1_N=((int16_t)I1_Abs0-I1_Abs)*100/Koef1; //12000; TEMP
  //(int16_t)I1_Abs0= Io_TA1;
   int32_t I1 = 0; // 79->AFB25   95->AFB40
   #ifdef AFB_40 
      I1 = (I1_Abs*95)/256; //изменение версии ПО
      //Koef2=30; //01_04_21 - СШ сказала что 4В это 100А для 4-ки
   #else
      I1 = (I1_Abs * 79) / 256; //изменение версии ПО //вернул 30_03_22
   #endif
  
  //I1_N=(int16_t)Io_TA1 - (I1/256); // >>8
  //I2_N=(int16_t)Io_TA2 - (I2_Abs/Koef2); //I2_Abs0 вносить поправку на напряжение смещения? 10мВ -2,5%
  I1_N = (Io_TA1&0x080)?(((int16_t)Io_TA1|0xff00) - I1):((int16_t)Io_TA1 - I1); // >>8  ///256
  I2_N = (Io_TA2&0x080)?(((int16_t)Io_TA2|0xff00) - (I2_Abs/Koef2)):((int16_t)Io_TA2 - (I2_Abs/Koef2)); //I2_Abs0 вносить поправку на напряжение смещения? 10мВ -2,5%
  
}

