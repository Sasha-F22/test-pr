
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "ADC.h"
#include "Modbus.h"

/* Private variables ---------------------------------------------------------*/
 int16_t I1_Abs;       // volatile ������ ��� ������� ������
 int16_t I2_Abs;       // volatile ������ ��� ������� ������
 int16_t I1_N;         //volatile��� 1 � �������
 int16_t I2_N;         // volatile ��� 2 � �������
 uint16_t time_s = 0;
 

 uint16_t f_ADC = 0;
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //

void ADC_Init(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);       // CS = 0 ������ ����
}

 ////////////////////////  ������������ �������� ��� �������/������ ���  ////////////////////
    /*  ������  	          	PG:8 - (RD)���������� ���������, �� �����;
                                PA:8 - (CS)�������������, 0 - ��������� ; 
                                PG:6 - (CONVST)������ ������-�, �� ������;
								PG:7 - (RST)

      �����                     PA:11 - (BUSY)�������������� ����;
                              	PA:12 - ������ ����� ���������;
    */
#pragma optimize = none
void ADC_Start(void)
{
  //TEMP - �������� ����� ���������� � 1 ��� �� 100 ��
  if(GPIOA->IDR & GPIO_PIN_11)  //test
	GPIOG->BSRR = GPIO_BSRR_BR6;
  GPIOG->BSRR = GPIO_BSRR_BR6;          // ������ ������. = 0
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
  GPIOG->BSRR = GPIO_BSRR_BS6; // ������ ������. = 1

  uint32_t fin = DWT -> CYCCNT;
  time_s = fin - start;
}

void ADC_Results(void)//���������� ���������� ����� �������������� 
{
  //TEMP - �������� ����� ���������� � 2,8 ��� �� 300 ��
  //1-� ����� ����
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
  //2-� ����� ����
  GPIOG->BSRR = GPIO_BSRR_BR8;                  // RD = 0
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");			// while (GPIOG->IDR & GPIO_PIN_8){} //����
  I2_Abs = GPIOD->IDR;                 
  I2_Abs &= (~0x04); 	//
  I2_Abs |= (GPIOG->IDR & GPIO_PIN_12) >> 10;
  GPIOG->BSRR = GPIO_BSRR_BS8;                  // RD = 1
  //__ASM("NOP");
  //__ASM("NOP");
  /*//3-� ����� ���
  GPIOG->BSRR = GPIO_BSRR_BR5;                  // RD = 0 
  __ASM("NOP");
  __ASM("NOP");                                                                                              
  GPIOG->BSRR = GPIO_BSRR_BS5;                  // RD = 1
  __ASM("NOP");
  __ASM("NOP");
  //4-� ����� ���
  GPIOG->BSRR = GPIO_BSRR_BR5;                  // RD = 0 
  __ASM("NOP");
  __ASM("NOP");
  GPIOG->BSRR = GPIO_BSRR_BS5;                  // RD = 1
  __ASM("NOP");
  __ASM("NOP");*/
}

void ADC_Reset(void)//����� �������� ��� ����� ���������
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET); // ����� ��� - �����
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  __ASM("NOP");
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET); // ����� ��� - ���� 
}

//const uint16_t Koef1=324; //317.5 ������
uint16_t Koef2 = 60; //300
//__no_init uint8_t I1_Abs0 @0x20000040; // �������� ��� ���������� ��� ����������� �� - ��������������� (��������)

void ADC_Norm(void)//������ � ������ ��������� ����
{
  //I1_Abs0=5;
  //I1_N=((int16_t)I1_Abs0-I1_Abs)*100/Koef1; //12000; TEMP
  //(int16_t)I1_Abs0= Io_TA1;
   int32_t I1 = 0; // 79->AFB25   95->AFB40
   #ifdef AFB_40 
      I1 = (I1_Abs*95)/256; //��������� ������ ��
      //Koef2=30; //01_04_21 - �� ������� ��� 4� ��� 100� ��� 4-��
   #else
      I1 = (I1_Abs * 79) / 256; //��������� ������ �� //������ 30_03_22
   #endif
  
  //I1_N=(int16_t)Io_TA1 - (I1/256); // >>8
  //I2_N=(int16_t)Io_TA2 - (I2_Abs/Koef2); //I2_Abs0 ������� �������� �� ���������� ��������? 10�� -2,5%
  I1_N = (Io_TA1&0x080)?(((int16_t)Io_TA1|0xff00) - I1):((int16_t)Io_TA1 - I1); // >>8  ///256
  I2_N = (Io_TA2&0x080)?(((int16_t)Io_TA2|0xff00) - (I2_Abs/Koef2)):((int16_t)Io_TA2 - (I2_Abs/Koef2)); //I2_Abs0 ������� �������� �� ���������� ��������? 10�� -2,5%
  
}

