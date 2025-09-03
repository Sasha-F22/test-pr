/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "Tiristor.h"

/*  Variables ---------------------------------------------------------*/
uint8_t VS1,VS2,VS3;    //Флаги состояния тиристоров VS1, VS2, VS3 -> 0-on, 1-off
uint8_t VS_Start = 0;   //Управление импульсом ускорения включения тиристоров (Время импульса в циклах)

uint8_t Ch_VS = 0;
uint8_t Ch_Tiristor;

uint16_t Ch_VS1 = 0;                    //Счетчик управления импульсом включения VS1
uint16_t Ch_VS1_Number_Impuls = 0;      //Счетчик управления количеством импульсов VS1 (команда старт)
uint16_t Ch_VS1_Period = 0;             //Счетчик управления периодом импульсов VS1

uint16_t Ch_VS2 = 0;                    //Счетчик управления импульсом включения VS2
uint16_t Ch_VS2_Number_Impuls = 0;      //Счетчик управления количеством импульсов VS2 (команда старт)
uint16_t Ch_VS2_Period = 0;             //Счетчик управления периодом импульсов VS2

uint16_t Ch_VS3 = 0;                    //Счетчик упавления импульсом включением VS3
uint16_t Ch_VS3_Number_Impuls = 0;      //Счетчик упавления количеством импульсов VS3 (команда старт)
uint16_t Ch_VS3_Period = 0;             //Счетчик упавления периодом импульсов VS3

#define VS1_PIN GPIO_PIN_13             //Тиристор прямого гасящего тока
#define VS2_PIN GPIO_PIN_14             //Тиристор обратного гасящего тока
#define VS3_PIN GPIO_PIN_15             //Катушка отключения
#define VS_PIN GPIO_PIN_10              //Вывод ускорения включения тиристоров

#define VS_Port GPIOB
#define VS1_Port GPIOE
#define VS2_Port GPIOE
#define VS3_Port GPIOE

#define TT_VS3_Period 40                //Заданный интервал в циклах / 200мкс
#define TT_VS3_Impuls 6                 //Заданный интервал в циклах / 30мкс

uint16_t Ch_VS3_Pauza = 0;
#define TT_VS3_Pauza 100                //Заданный интервал в циклах

#define TT_VS2_Period 10                //Заданный интервал в циклах / 50мкс
#define TT_VS2_Impuls 6                 //Заданный интервал в циклах / 30мкс

uint16_t Ch_VS2_Pauza = 0;
#define TT_VS2_Pauza 100                //Заданный интервал в циклах

#define IGCT_On GPIOE->BSRR = GPIO_BSRR_BR1
#define IGCT_Off GPIOE->BSRR = GPIO_BSRR_BS1

uint8_t Gash_Duge=0;
uint16_t TTT=0;
uint16_t Ch_Imp=0;
extern uint8_t Fault_25;
extern uint8_t Faultec;

/* Function prototypes -----------------------------------------------*/


uint8_t Imp_Uskor=0;
extern uint8_t F8_Proverka;
extern uint8_t F0_Proverka;
extern uint8_t Opto_IN1;
static uint16_t Ch_Opto1;
extern uint8_t F_Vkl_Prov;
extern uint8_t FAZA_Otkl;
extern uint16_t T_Otkl;
extern uint8_t f_Rozr ;
extern uint8_t Ch_Out_X4_7 ; //выход блокировки,для паралельной работы БВ
extern uint8_t Out_X4_7;
extern uint16_t Ch_UART1;

void Tiristor (void) //5mks
{   

  ////////////////////////////////////////////////////////////////////////////////
  if ( Gash_Duge !=0 )
  {   Ch_UART1=0; //02_11_20
      // GPIOE->BSRR = GPIO_BSRR_BS1;  //TEMP //TEMP
      if ( Gash_Duge==1 ) switch (TTT)//для рисунка 1
      {
      case 0:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  Formirov_Imp_Tiristorov (3,1);
      case 1:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  Formirov_Imp_Tiristorov (3,1);
      case 240:  VS1=0; VS2=1; VS3=0; Ch_Imp=3; Imp_Uskor=1; break; //240  VS2-3 //240 //285
      case 370:  VS1=1; VS2=0; VS3=0; Ch_Imp=4; Imp_Uskor=1; break; //360  VS1-5 //360  //428
      case 510:  VS1=0; VS2=1; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //510  VS1-7 //510  //607
      case 650:  VS1=1; VS2=0; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //650  VS2-7 //650  //773
      case 760:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; Out_X4_7=0; break; //740 //880
      }
      
      if ( Gash_Duge==2 ) switch (TTT)//для рисунка 2
      {
      case 0:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  
      case 1:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  
      case 140:  VS1=0; VS2=1; VS3=0; Ch_Imp=4; Imp_Uskor=1; break; //240  VS2-3 //140
      case 270:  VS1=1; VS2=0; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //360  VS1-7 //270
      case 370:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; Out_X4_7=0; break; // 360
      }
      
      if ( Gash_Duge==3 ) switch (TTT)//для рисунка 3
      {
      case 0:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  
      case 1:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //0  VS3-1  
      case 110:  VS1=0; VS2=1; VS3=0; Ch_Imp=5; Imp_Uskor=1; break; //240  VS2-5//100
      case 280:  VS1=1; VS2=0; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //360  VS1-7//270
      case 370:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; Out_X4_7=0; break; //360
      }
      
      if ( Gash_Duge==4 ) switch (TTT)//для рисунка 4
      {
      case 0:    VS1=0; VS2=1; VS3=0; Ch_Imp=5; Imp_Uskor=1; break;   
      case 1:    VS1=0; VS2=1; VS3=0; Ch_Imp=5; Imp_Uskor=1; break;   
      case 80:   VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //80
      case 320:  VS1=1; VS2=0; VS3=0; Ch_Imp=4; Imp_Uskor=1; break; //300
      case 450:  VS1=0; VS2=1; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //410
      case 570:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; Out_X4_7=0; break; //530
      }
      
      if ( Gash_Duge==5 ) switch (TTT)//для рисунка 5
      {
      case 0:    VS1=0; VS2=1; VS3=0; Ch_Imp=5; Imp_Uskor=1; break;   
      case 1:    VS1=0; VS2=1; VS3=0; Ch_Imp=5; Imp_Uskor=1; break;   
      case 80:   VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break; //80
      case 220:  VS1=1; VS2=0; VS3=0; Ch_Imp=4; Imp_Uskor=1; break; //220
      case 350:  VS1=0; VS2=1; VS3=0; Ch_Imp=8; Imp_Uskor=1; break; //330
      case 450:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; Out_X4_7=0; break; // было 450 //430
      }
      
      /////для тестового режима //02_03_20
      if ( Gash_Duge==6 ) switch (TTT)//для 
      {
      case 0:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break;   
      case 1:    VS1=0; VS2=0; VS3=1; Ch_Imp=1; Imp_Uskor=1; break;  
      case 110:  VS1=0; VS2=0; VS3=0; Gash_Duge=0; TTT=0; Ch_Imp=0; FAZA_Otkl=1; T_Otkl=0; break; // было 450 //430
      }
      
      uint8_t Impuls_Tirist = TTT%10;
      if ((Impuls_Tirist==9) && (Ch_Imp!=0))Ch_Imp--; //50mks счёт импульсов
  
      if ((Ch_Imp!=0)&&(Impuls_Tirist<6)) //импульс 30мксек
      {
        if (Imp_Uskor){GPIOB->BSRR = GPIO_BSRR_BS10;}
        GPIOE->BSRR = (0x1U&&VS1) << 13; // if (VS1){ GPIOE->BSRR = GPIO_BSRR_BS13; } // VS1 ? GPIOE->BSRR = GPIO_BSRR_BS13 : __ASM("NOP");
        GPIOE->BSRR = (0x1U&&VS2) << 14; //GPIOE->BSRR = (0x1U&&VS2) << 14; // GPIOE->BSRR = VS2 ?  GPIO_BSRR_BS14 : GPIO_BSRR_BR14; // if (VS2){ GPIOE->BSRR = GPIO_BSRR_BS14; }
        GPIOE->BSRR = (0x1U&&VS3) << 15; //if (VS3){ GPIOE->BSRR = GPIO_BSRR_BS15; } // VS3 ? GPIOE->BSRR = GPIO_BSRR_BS15 : __ASM("NOP");
        // или так GPIOE->BSRR = ((0x1U&&VS1) << 13)|((0x1U&&VS2) << 14)|((0x1U&&VS3) << 15);
      }
      else { GPIOB->BSRR = GPIO_BSRR_BR10; GPIOE->BSRR = GPIO_BSRR_BR13; GPIOE->BSRR = GPIO_BSRR_BR14; GPIOE->BSRR = GPIO_BSRR_BR15; Imp_Uskor=0;} //GPIOE->BSRR = 0xE0000000;
     
      TTT++;
       // GPIOE->BSRR = GPIO_BSRR_BR1; //TEMP //TEMP / 1мкс
      return;  
  }
  
  ///////////////////////
  if(F8_Proverka==2) //пачка имп-в для проверки VD2 
  {   Ch_UART1=0; //02_11_20
      switch (TTT)//для проверки
      {
      case 0:     VS1=0; VS2=1; VS3=0; Ch_Imp=160; break; //
      case 1:     VS1=0; VS2=1; VS3=0; Ch_Imp=160; Ch_Opto1=0; break; //
      case 1600:  VS1=0; VS2=0; VS3=0; //F8_Proverka=3; TTT=0; Ch_Imp=0; // 15_03_21 по письму СШ  
                  if (Ch_Opto1 < 400) 
                  {Fault_25 = 1; Faultec=1; }  break; //F8_Proverka = 0;F1_KQF1 = 0;8мсек прошло
      case 2000:  F8_Proverka=3; TTT=0; Ch_Imp=0; break;  //15_03_21 по письму СШ        
      }
      
      uint8_t Impuls_Tirist = TTT%10; //
      if ((Impuls_Tirist==9)&&(Ch_Imp!=0))Ch_Imp--; //50mks счёт импульсов
  
      if ((Ch_Imp!=0)&&(Impuls_Tirist<6)) //импульс 30мксек
      {
         GPIOE->BSRR = GPIO_BSRR_BS14; 
         //IGCT_On; //TEMP  vd2 proverka //закомментил 27_02
         Ch_Opto1+= (Opto_IN1==0); //
      }
      else { 
        GPIOB->BSRR = GPIO_BSRR_BR10; 
        GPIOE->BSRR = GPIO_BSRR_BR13; 
        GPIOE->BSRR = GPIO_BSRR_BR14; GPIOE->BSRR = GPIO_BSRR_BR15; IGCT_Off;} //TEMP 
     
      TTT++;
  }
  
  if (F8_Proverka == 6) // Для AFB_25
  {   Ch_UART1=0; //02_11_20
    switch (TTT)//для проверки при вкл 
      {
      case 0:     VS1=0; VS2=1; VS3=0; Ch_Imp=20; break; //
      case 1:     VS1=0; VS2=1; VS3=0; Ch_Imp=20; break; //
      case 200:  VS1=0; VS2=0; VS3=0;  break; //
      case 2000:  VS1=0; VS2=0; VS3=0;  TTT=0; F8_Proverka =7; Ch_Imp=0; break; //F_Vkl_Prov=3;
      }
      
      uint8_t Impuls_Tirist = TTT%10; //замерить скорость
      if ((Impuls_Tirist==9)&&(Ch_Imp!=0))Ch_Imp--; //50mks счёт импульсов
  
      if ((Ch_Imp!=0)&&(Impuls_Tirist<6)) //импульс 30мксек
      {
         GPIOE->BSRR = GPIO_BSRR_BS14; 
      }
      else  GPIOE->BSRR = GPIO_BSRR_BR14;  //
     
      TTT++;
  }
  
  if(f_Rozr == 1)
  {   Ch_UART1=0; //02_11_20
    switch (TTT)//для проверки при вкл
      {
      case 0:      Ch_Imp=20; break; //
      case 1:      Ch_Imp=20; break; //
      case 200:    break; //
      case 300:   TTT=0; f_Rozr =0; Ch_Imp=0; break; //
      }
      uint8_t Impuls_Tirist = TTT%10; 
      if ((Impuls_Tirist==9)&&(Ch_Imp!=0))Ch_Imp--; //50mks счёт импульсов
  
      if ((Ch_Imp!=0)&&(Impuls_Tirist<6)) //импульс 30мксек
      {
         GPIOE->BSRR = GPIO_BSRR_BS14 | GPIO_BSRR_BS13;
      }
      else  GPIOE->BSRR = GPIO_BSRR_BR14 | GPIO_BSRR_BR13;  //
     
      TTT++;
  }											   
} 