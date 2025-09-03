


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Control.h"

/*  Variables ---------------------------------------------------------*/
uint16_t Contr_Pin=0; //����� ������������
uint8_t f_Akum_=0;     //���� ����� Li-ion ������������
uint8_t f_LEM_Con=0;   // ������ �������� ���� ���������
uint8_t f_Supply_V_15=0; //������� LEM �������� +-15�
uint8_t f_Tirist_OK=0; //������� ����� ���������� �����������
uint8_t f_Low_24V =0; //����. ���� ������� (�� �����) ���� 21� //Fault_05
uint8_t f_X4B_Con =0; //������ �4� ���������
uint8_t f_Res_1 =0;   //�������� ������������
uint8_t Fault_06 =0;
uint8_t Fault_05 =0;

//extern uint8_t Fault_04;
//extern uint8_t Fault_03;
uint8_t Fault_16 =0;
///���������� ���������� �����
uint8_t Opto_IN1=0; //���������� ���� �������� ������ "D in 2"
uint8_t Opto_IN2=0; //���������� ���� �������� IGCT (4�� ������) "D in 1"
uint8_t Opto_IN3=0; //���������� ���� ����� ���-�� "D in 3"
uint8_t Opto_Out=0; //���� ���?
extern uint8_t f_Zapusk;
//uint8_t Ch_F_16=0;
extern  uint8_t Faultec ;
extern  uint8_t f_SPI1_ ;

/* Function -------------------------------------------------------------*/

void Contr_Deb(void) //����������� �����. �������
{
    static uint16_t  cntr0, cntr1 , cntr2;
    uint16_t delta, sample;
    //��������
    /*sample = (((((GPIOE->IDR) & GPIO_PIN_2) | ((GPIOA->IDR)& GPIO_PIN_11)) | (((GPIOA->IDR)& GPIO_PIN_12) | ((GPIOC->IDR)& GPIO_PIN_4))) 
      | (((GPIOC->IDR)& GPIO_PIN_5) | (((GPIOB->IDR)& GPIO_PIN_11)>>1))) | (((((GPIOE->IDR)& GPIO_PIN_5)<<1)
    |(((GPIOC->IDR)& GPIO_PIN_11)>>2)) |((((GPIOC->IDR)& GPIO_PIN_12)<<1) |(((GPIOG->IDR)& GPIO_PIN_2)>>1))); */
    sample = ((GPIOC->IDR) & (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5));       //����������� ����-������ 
    sample |= ((GPIOA->IDR) & (GPIO_PIN_13 | GPIO_PIN_15)); //sample |= ((GPIOA->IDR)& GPIO_PIN_11); sample |= ((GPIOA->IDR)& GPIO_PIN_12);
    sample |= (((GPIOE->IDR) & (GPIO_PIN_1 ))>>1); //sample |= ((GPIOC->IDR)& GPIO_PIN_4); sample |= ((GPIOC->IDR)& GPIO_PIN_5); 
    sample |= ((GPIOB->IDR) & (GPIO_PIN_10));      
    sample |=(((GPIOB->IDR)& GPIO_PIN_4)<< 2);    //sample |=(((GPIOE->IDR)& GPIO_PIN_5)<<1);
    //sample |= (((GPIOC->IDR)& GPIO_PIN_11)>>2);   sample |= (((GPIOC->IDR)& GPIO_PIN_12)<<1);    sample |=(((GPIOG->IDR)& GPIO_PIN_2)>>1);
    
    delta = sample ^ Contr_Pin;
    cntr2 = (cntr2 ^ (cntr1 & cntr0)) & delta;
    cntr1 = (cntr1 ^ cntr0) & delta;
    cntr0 = ~cntr0 & delta;
    
    Contr_Pin ^= (cntr0 & cntr1 & cntr2); 
 
    //return Contr_Pin;
    //����������
    f_Akum_ = ((Contr_Pin & GPIO_PIN_6) != 0);
    f_LEM_Con = (Contr_Pin & GPIO_PIN_0) != 0; 
    f_Supply_V_15 = (Contr_Pin & GPIO_PIN_15)!=0; 
    f_Tirist_OK = (Contr_Pin & GPIO_PIN_10)!=0; // ������ ���-�?
    f_Low_24V = (Contr_Pin & GPIO_PIN_5)!=0; 	//Fault_05
    f_X4B_Con = (Contr_Pin & GPIO_PIN_4)!=0; //������� ������ �����.
    //f_Res_1 = (Contr_Pin & GPIO_PIN_6)!=0; //?
    

   /* //������� � �������� ����� 01_04_21
    if (f_Low_24V && (f_Zapusk==0)){ 
      Ch_F_16++; 
      if (Ch_F_16>8) { 
        if((Fault_16==0)&&((inp1_1==1)&&(inp1_2==0)&&(inp1_3==1)&&(inp1_4==0))) �12_4 =1;//
        Fault_16 =1; Faultec = 1;
      } 
    } 
    else Ch_F_16=0;*/
      
    Fault_06 |= ((  f_LEM_Con || f_Supply_V_15 || f_Tirist_OK || f_X4B_Con || f_SPI1_)&&(f_Zapusk==0));
    //Fault_04 |= (( f_Akum_==0)&&(f_Zapusk==0)); //����������� ��� ������ �������
    Opto_IN1 = ((Contr_Pin & GPIO_PIN_1)!=0);  //11-� ��� ������� �� ��� ������//��� UART4 RX
    Opto_IN2 = ((Contr_Pin &  GPIO_PIN_2)!=0); //12-� ��� ������� �� ���� �����
    Opto_IN3 = ((Contr_Pin &  GPIO_PIN_3)!=0); //2-� ��� ������� �� ���� ������ 
    
}

/*void Contr_Deb_a (void)
{
  Akum_ = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
  LEM_Con = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  Supply_V_15 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  Tirist_OK = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
  Low_24V = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
  X4B_Con = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11); //DI_DO?
  Res_1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5); //��������� ������������
  //Res_2 = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1); //��������� ������������
  //Res_3 = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3); //��������� ������������

  Opto_IN1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11); //UART4 RX
  Opto_IN2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
  Opto_IN3 = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);
  //Opto_Out = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);

  
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)) Akum_++;
  else Akum_=0;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) LEM_Con++; //� ���
  else LEM_Con=0;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) Supply_V_15++;//� ���
  else Supply_V_15=0;
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) Tirist_OK++ ;//� ���������
  else Tirist_OK=0;
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)) Low_24V++;
  else Low_24V=0;
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) X4B_Con++; //DI_DO?
  else X4B_Con=0;
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)) Res_1++; 
  else Res_1=0;
   
}*/




