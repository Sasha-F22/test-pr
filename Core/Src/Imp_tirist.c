
/* Includes ------------------------------------------------------------------*/
#include "Imp_tirist.h"
#include "stm32h7xx_it.h"
//#include "stm32h735xx.h"
#include "stm32h7xx_hal.h"
/* Declarations and definitions ----------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
uint8_t f_fr_t = 0;
uint8_t Vs1 = 0, Vs2 = 0, Vs3 = 0; 
uint8_t Nimp1 = 0, Nimp2 = 0, Nimp3 = 0;

uint8_t Gash_Duge;
extern uint8_t Ch_Tir;

extern TIM_HandleTypeDef htim3;

/* Functions -----------------------------------------------------------------*/

void Imp_tirist (void) // 20mks
{
  if(f_fr_t) 
  {
    //f_fr_t = 0;
    Vs1 = 0; Vs2 = 0; Vs3 = 0; //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	//GPIOE->BSRR = GPIO_BSRR_BR13; GPIOE->BSRR = GPIO_BSRR_BR14; GPIOE->BSRR = GPIO_BSRR_BR15;
    if(Nimp1 != 0) Nimp1 --;
    if(Nimp2 != 0) Nimp2 --;
    Nimp3 = 0; //if(Nimp3 != 0) Nimp3 --; 
  }
  else
  {
    Vs1 = (Nimp1 != 0); //GPIOE->BSRR = (0x1U&&(Nimp1 != 0)) << 13;
    Vs2 = (Nimp2 != 0); //GPIOE->BSRR = (0x1U&&(Nimp2 != 0)) << 14;
    //Vs3 = (Nimp3 != 0);
  }
  
  f_fr_t ^= 1;
  //HAL_TIM_Base_Stop_IT(&htim3);
}

void Gash_Dug (void) // 50mks
{
  if(Gash_Duge == 1)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 24) 
	{ 
	  Nimp2 = 3; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 37) 
	{ 
	  Nimp1 = 4; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
	if(Ch_Tir == 51) 
	{ 
	  Nimp2 = 8; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
	if(Ch_Tir == 65) 
	{ 
	  Nimp1 = 8; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    //
  }
  
  if(Gash_Duge == 2)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 14) 
	{ 
	  Nimp2 = 4; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 27) 
	{ 
	  Nimp1 = 4; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    
  }
  
  if(Gash_Duge == 3)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 11) 
	{ 
	  Nimp2 = 4; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 28) 
	{ 
	  Nimp1 = 4; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    
  }
  
  /////////////////////////// те что ниже ещё не правил
  if(Gash_Duge == 4)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp2 = 5; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 8) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 32) 
	{ 
	  Nimp1 = 4; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
	if(Ch_Tir == 45) 
	{ 
	  Nimp2 = 8; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    
  }
  
  if(Gash_Duge == 5)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp2 = 5; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 8) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    if(Ch_Tir == 22) 
	{ 
	  Nimp1 = 4; Vs1 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 13;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
	if(Ch_Tir == 35) 
	{ 
	  Nimp2 = 8; Vs2 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 14;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    
  }
  
  if(Gash_Duge == 6)
  {
    if(Ch_Tir == 0 ) 
	{ 
	  Nimp3 = 1; Vs3 = 1; f_fr_t = 1; 
	  GPIOE->BSRR = (0x1U) << 15;
	  HAL_TIM_Base_Start_IT(&htim3);
	}
    
  }
  
}

/*----------------------------------------------------------------------------*/






//////// The end ///////