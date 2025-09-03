/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32h735xx.h"
#include "stm32h7xx_hal.h"
#include "UART1.h"
#include "Modbus.h"

/* ------- variables ---------------------------------------------------------*/

uint8_t  Ch_Nach_P = 0; 	//Счёт сбоев начала пакета - от изм-ля напр.
uint8_t   f_UART1 = 0; 		//Флаг ошибка непринятых пакетов UART1 - от изм-ля напр.
uint8_t  f_Nach_Pac = 0;	//Флаг ошибка начала пакета - от изм-ля напр.
uint8_t   f_CRC_UART1 = 0; 	//Флаг ошибка CRC проверки
uint8_t  Ch_CRC_UART1 = 0;
uint8_t  RxSize = 0; 		//Счёт принятых байтов UART1 - от изм-ля напр.
uint16_t Ch_UART1 = 0; 		//Счёт непринятых пакетов UART1 - от изм-ля напр.
uint16_t aRxBuffer[16]; 	//буфер приема данных от Измерителя напряжения
uint16_t Ch_UART8 = 0;
uint16_t  f_UART8 = 0;
 
uint16_t V1_Abs,V2_Abs,V3_Abs;
int16_t  V1_N,V2_N,V3_N;
uint16_t V1_Koeff = 0;
uint16_t V2_Koeff = 0;
uint16_t V3_Koeff = 0;

extern UART_HandleTypeDef huart1;
uint8_t V1L,V1H,V2L,V2H,V3L,V3H,Crc_;
//extern uint8_t STATUS;

// Таблица CRC
const uint8_t Crc8Table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC };

//// функция CRC
uint8_t Crc8(uint8_t  *pcBlock, uint8_t  len)
{
    uint8_t crc = 0xff;
 
    while (len--)
        crc = Crc8Table[crc ^ *(pcBlock++)];       
        
    return crc;
}


void CRC_Controll (uint8_t Kanal_V)
{ 
	///////// расчёт CRC
  
	//if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))V2L=0x04;//для проверки ЦРЦ испортим один бит
	uint8_t s;    
    /*uint8_t  pcBlock[6] ={V1L,V1H,V2L,V2H,V3L,V3H}, len = 6; // uint8_t  pcBlock[8] ={V1L,V1H,V2L,V2H,V3L,V3H}, len = 6
    s = Crc8(pcBlock, len); // 0,6мкс CRC байт  */
    
    s = 0xff;
    s = Crc8Table[s ^ V1L];
    s = Crc8Table[s ^ V1H];
    s = Crc8Table[s ^ V2L];
    s = Crc8Table[s ^ V2H];
    s = Crc8Table[s ^ V3L];
    s = Crc8Table[s ^ V3H];
    
	if (Crc_ == s)
    { 
       Ch_CRC_UART1 = 0;
       Ch_UART1 = 0;
        f_UART1 = 0;
       //GPIOE->BSRR = GPIO_BSRR_BR1; //temp
       ////Абсолютное и нормированое значение
		if(Kanal_V == 1) {
      V1_Abs= ((V1H & 0x7f)<<7) | ((V1L & 0x7f));
      if(V1H & 0x40){
        V1_Abs |= 0xC000; 
        V1_Abs = ~V1_Abs;
        V1_N = ((V1_Abs * V1_Koeff)>>11); //V1_Abs<<7/425; 
        V1_N = ~V1_N ; }
	  else V1_N = (V1_Abs * V1_Koeff)>>11;}
  
      if(Kanal_V == 2) {V2_Abs= ((V2H & 0x7f)<<7) | ((V2L & 0x7f));
      if(V2H & 0x40){
        V2_Abs |= 0xC000; 
        V2_Abs = ~V2_Abs;
        V2_N = (V2_Abs * V2_Koeff)>>11;
        V2_N = ~V2_N ; }
	  else V2_N = (V2_Abs * V2_Koeff)>>11;}
      //V2_N = 900 ;
  
      /*V3_Abs= ((V3H & 0x7f)<<7) | ((V3L & 0x7f));
      if(V3H & 0x40){
        V3_Abs |= 0xC000; 
        V3_Abs = ~V3_Abs;
        V3_N = (V3_Abs * V3_Koeff)>>8;  //нормированное значение *8
        V3_N = ~V3_N ; }
      else V3_N = (V3_Abs * V3_Koeff)>>8;*/ //нормированное значение *8
      
      ////////////////////  25_08_20  //////////////////
      if(STATUS > 2){ //для режима AFB_N 28_09_20
        V1_N = (~V1_N) ; V2_N = (~V2_N) ; V3_N = (~V3_N) ;
      }
      /////////////////////////////////////////////////
    }
    else 
    { 
        Ch_CRC_UART1 ++;
        if (Ch_CRC_UART1 > 0x1f) {
            f_CRC_UART1 = 1;
            V1L = 0; V1H = 0; V2L = 0; V2H = 0; V3L = 0; V3H = 0;
        }
    }
        
}


///////////новый обраб-к прер-й уарт
void HAL_UART1_IRQ(UART_HandleTypeDef *huart)
{
   uint32_t isrflags   = READ_REG(huart->Instance->ISR);
   uint32_t cr1its     = READ_REG(huart->Instance->CR1);
   uint32_t cr3its     = READ_REG(huart->Instance->CR3);
   uint32_t errorflags = 0x00U;
   //uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if(errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE_RXFNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      //UART_Receive_ITD(huart);
      return;
    }
  }
}


void UART_Receive_ITD(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;
  
  // Check that a Rx process is ongoing 

  if(huart->RxState == HAL_UART_STATE_BUSY_RX) 
  {
    tmp = (uint16_t*) huart->pRxBuffPtr;
    *tmp = (uint16_t)(huart->Instance->RDR & (uint16_t)0x01FF);
    huart->pRxBuffPtr += 2U;
    
    if(--huart->RxXferCount == 0U)
    {
      // Disable the UART Parity Error Interrupt and RXNE interrupt//
      CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      // Disable the UART Error Interrupt: (Frame error, noise error, overrun error) //
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      // Rx process is completed, restore huart->RxState to Ready //
      huart->RxState = HAL_UART_STATE_READY;
     
      //HAL_UART_RxCpltCallback(huart);
      //return HAL_OK;
    }
    //return HAL_OK;
  }
  //else{return HAL_BUSY;}
}


void GPIO_PA9_Init( )
{
  	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    GPIO_InitStruct.Pin  = GPIO_PIN_9;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
}