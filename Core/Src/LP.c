
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "LP.h"
//#include "OLeD.h"

//#include "Tiristor.h"//TEMP
#include "Algoritm.h"//TEMP
#include "Modbus.h"

/* ------- variables ---------------------------------------------------------*/
extern uint8_t F8_QS1;//TEMP
extern uint8_t F8_QF1;//TEMP
extern uint8_t F0_KM1;//TEMP
extern uint8_t F8_KM1;//TEMP
extern uint16_t T_LCD_E;
extern uint8_t F_Over_Current;
extern uint8_t Otkaz_IKZ ;
extern uint8_t F_AFB_Good;
extern uint32_t FAULT;

volatile uint8_t Key_Data = 0;

uint8_t Key_Init = 0; //��������� ������ ������������������� (������������ ��� ������)

uint8_t Key_Lft = 0;
uint8_t Key_Rgt = 0;
uint8_t Key_Upp = 0;
uint8_t Key_Dwn = 0;
uint8_t Key_Ent = 0;

uint8_t Key_Lft_Old = 0;
uint8_t Key_Rgt_Old = 0;
uint8_t Key_Upp_Old = 0;
uint8_t Key_Dwn_Old = 0;
uint8_t Key_Ent_Old = 0;

uint8_t f_Test = 0;
uint8_t Fault_Menu = 0;
uint8_t Ch_Fault_Menu = 0;
uint8_t Regul_UpDown = 0;
uint8_t Ch_Test_Menu = 0;
uint16_t Ch_Regul_UpDown = 0;
uint8_t F_Pisk = 0;

uint8_t KeyDeb(void ) //������ � �������������, ��������� state
{
    static unsigned char  cnt0, cnt1 , cnt2;
    unsigned char delta, sample;
    
    sample = ( (GPIOB->IDR)>>5 ) & 0x1F;
    
    if (!Key_Init)      //������������� �������� ��� ������ �������
    {
      Key_Data = sample;
      Key_Init = 1;
    }
       
    delta = sample ^ Key_Data;
    cnt2 = (cnt2 ^ (cnt1 & cnt0)) & delta;
    cnt1 = (cnt1 ^ cnt0) & delta;
    cnt0 = ~cnt0 & delta;

    Key_Data ^= (cnt0 & cnt1 & cnt2);
 
    return Key_Data;
}


#define inp1_1  (Input_X1 & 0x0001)
#define inp1_2 ((Input_X1 & 0x0002)>>1)
#define inp1_3 ((Input_X1 & 0x0004)>>2) //�� ��������
#define inp1_4 ((Input_X1 & 0x0008)>>3) //�� �������

void Key_Status(void) //10ms
{
  KeyDeb();
  //������ ��������� ������
  Key_Upp = (Key_Data >> 0) & 0x01; //up
  Key_Lft = (Key_Data >> 3) & 0x01; //left
  Key_Dwn = (Key_Data >> 4) & 0x01; //down
  Key_Rgt = (Key_Data >> 1) & 0x01; //rgt
  Key_Ent = (Key_Data >> 2) & 0x01;
  
  if ((Key_Lft != Key_Lft_Old) && (Key_Lft == 0))
  {
    Knob_Prev (); F_Pisk = 1;
	GPIOF->BSRR = GPIO_BSRR_BR8;
    /*������ ������*/
    //F8_QS1 = 1; //��� �������� TEMP
    //T_LCD_E = 10000; //ssd1306_SetDisplayOn(1);
    //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);//���������
  }
  
  if ((Key_Rgt != Key_Rgt_Old)&&(Key_Rgt == 0))
  {
    Knob_Next (); F_Pisk = 1;
	GPIOF->BSRR = GPIO_BSRR_BR8;
    /*������ ������*/
    //T_LCD_E = 10000;
    //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    //F8_QS1 = 2; //��� �������� TEMP
  }
  
  if ((Key_Upp != Key_Upp_Old)&&(Key_Upp == 0))
  {
    Knob_Uppe (); F_Pisk = 1;
	GPIOF->BSRR = GPIO_BSRR_BR8;
    /*������ ������*/
    //T_LCD_E = 10000;
    //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    //F8_QF1 = 1; //��� �������� TEMP
  }
  
  if ((Key_Dwn != Key_Dwn_Old)&&(Key_Dwn == 0))
  {
    Knob_Down (); F_Pisk = 1;
	GPIOF->BSRR = GPIO_BSRR_BR8;
    /*������ ������*/
    //T_LCD_E = 10000;
    //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    //F8_QF1 = 2; //��� �������� TEMP
  }
  
  if ((Key_Ent != Key_Ent_Old)&&(Key_Ent == 0))
  {
    Knob_Set (); F_Pisk = 1;
	GPIOF->BSRR = GPIO_BSRR_BR8;
    /*������ ������*/
        
  }

  //������� � ����� ���� ��� ������-� ������� 2-� ������
  if(((Key_Rgt | Key_Lft)==0))
  {
    if (Ch_Fault_Menu > 170) {Fault_Menu = (Fault_Menu==0); Ch_Fault_Menu=0;}
    else Ch_Fault_Menu ++;
  }
  else Ch_Fault_Menu=0;
  
  //������� � ����� ���� ��� ������-� ������� 2-� ������ 28_02_20 
  //������ ���� �� � ������-�� ���������
  if(((Key_Upp | Key_Dwn)==0)&&((inp1_1==1)&&(inp1_2==0)&&(inp1_3==1)&&(inp1_4==0)))
  {
    if (Ch_Test_Menu > 170) {f_Test ^= 1; Ch_Test_Menu=0;}
    else Ch_Test_Menu ++;
  }
  else Ch_Test_Menu=0;  
  
  //���������� ��������� ������
  Key_Lft_Old = Key_Lft;
  Key_Rgt_Old = Key_Rgt;
  Key_Upp_Old = Key_Upp;
  Key_Dwn_Old = Key_Dwn;
  Key_Ent_Old = Key_Ent;
  
  //���������� ������� (��� �����������)-?
  if(Key_Upp == 0 | Key_Dwn == 0) Ch_Regul_UpDown ++;
  else Ch_Regul_UpDown = 0;
  if(Ch_Regul_UpDown > 200) {
    if(Regul_UpDown == 10) Regul_UpDown = 100;
    else Regul_UpDown = 10;
    Ch_Regul_UpDown = 0;
  }
  

  
  //���������� //
  //LED1-PC0, LED2-PF10, LED3-PC8, LED4-PB2, 
  GPIOC->BSRR = (F_AFB_Good)|((Otkaz_IKZ|f_Test )<< 8); // GPIO_BSRR_BS0 | GPIO_BSRR_BS8; 1-red
  GPIOC->BSRR = ((F_AFB_Good == 0) << 16)|(((Otkaz_IKZ|f_Test) == 0)<< 24); //
  GPIOF->BSRR = (F_Over_Current << 10); // GPIO_BSRR_BS10
  GPIOF->BSRR = ((F_Over_Current == 0) << 26);
  /*GPIOF->BSRR = (F_Over_Current << 11)|((Otkaz_IKZ|f_Test )<< 12)|((FAULT!=0) << 13); // �������� ����������
  GPIOF->BSRR = ((F_Over_Current==0) << 27)|(((Otkaz_IKZ|f_Test)==0) << 28)|((F_AFB_Good) << 29);// ����� ����������*/
  if (FAULT != 0) GPIOB->BSRR = GPIO_BSRR_BS2;          //PB2 = 1
  else GPIOB->BSRR = GPIO_BSRR_BR2;                     //PB2 = 0
  
}

