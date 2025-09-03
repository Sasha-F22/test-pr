/* RxDI-SN65HVS881; TxDO,RxDO-MAX14900EAGM; 
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32h735xx.h"
#include "stm32h7xx_hal.h"
#include "DI_DO.h"


/*  Variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern uint8_t Fault_06;
extern uint8_t f_NSS1;


__IO uint16_t fPin=0;   //буфер приема ДискрВх, после антидребезга, fPin&0x0200 - контроль воткн. Х1_1
//int8_t Vhod[10];      //буфер приема ДискрВх, до антидребезга, все - не нужен
uint8_t TxDO[6] = {0x54,0x00,0x00,0xaa,0x00,0x00}; //буфер передачи ДискрВых (Вых1,Настр1,CRC1,Вых2,Настр2,CRC2)
uint8_t RxDO[6] = {0x00,0x00,0x00,0x00,0x00,0x00}; //буфер ответа ДискрВых - данные по перегреву, перенапр, перегрузу и вотткн. разъёма
uint8_t RxDI[2] = {0x00,0x00}; //буфер приема ДискрВх
uint16_t deb_RxDI = 0;  //дискр вх Х1 после антидребезга

///выходы///
 
uint8_t Out_X1_1=0, Out_X1_2=0, Out_X1_3=0, Out_X1_4=0, Out_X1_5=0, Out_X1_6=0, Out_X1_7=0, Out_X1_8=0; //Устанавливаются в алгоритмах
uint8_t Out_X4_1=0, Out_X4_2=0, Out_X4_3=0, Out_X4_4=0, Out_X4_5=0, Out_X4_6=0, Out_X4_7=0, Out_X4_8=0; //Устанавливаются в алгоритмах


///входы///
 
//uint8_t In_X1_1=0, In_X1_2=0, In_X1_3=0, In_X1_4=0, In_X1_5=0, In_X1_6=0, In_X1_7=0, In_X1_8=0;//
//uint8_t In_X4_1=0, In_X4_2=0, In_X4_3=0, In_X4_4=0, In_X4_5=0, In_X4_6=0, In_X4_7=0, In_X4_8=0;//

uint8_t Ch_F = 0;

uint16_t Input_Optron(void) //антидребезг оптроны
{
    static uint16_t  cnt0, cnt1 , cnt2 , cnt3;
    uint16_t delta = 0, sample = 0;
    //int1-PB10,int2-PC4,int3-PC13,int4-PA0; (int2-ConectContr)
	//IN1-PE3,IN2-PE4,IN3-PE5,IN4-PE6,IN5-PE7,IN6-PE8,IN7-PE9,IN8-PE10,IN9-PE11,IN10-PE12
    sample = ((GPIOE->IDR) & 0x1FF8); 					// 3-12
	sample = sample | ((GPIOA->IDR) & 0x0001); 			// 0
	sample = sample | ((GPIOB->IDR) & 0x0400) >> 9; 	// 1
	sample = sample | ((GPIOC->IDR) & 0x2000);  		// 13
	sample = sample | ((GPIOC->IDR) & 0x0010) >> 2;		// 2
    sample = (~sample)&0x3FFF; //(~sample)&0x3FF
    
    delta = sample ^ fPin;
    cnt3 = (cnt3 ^ (cnt2 & cnt1 & cnt0)) & delta; //10_05
    cnt2 = (cnt2 ^ (cnt1 & cnt0)) & delta;
    cnt1 = (cnt1 ^ cnt0) & delta;
    cnt0 = ~cnt0 & delta;
    
    fPin ^= (cnt0 & cnt1 & cnt2 & cnt3);
 
    if ((0x04 & fPin) == 0) { //0x0200 & fPin
      if (Ch_F > 32) { Fault_06 = 1;  } //27_01_21 - откл. с разъед. fPin|=0x04; 20_07_22 
      else Ch_F++; 
    } 
    else Ch_F = 0;
    return fPin;
}

extern __IO uint16_t SPI2_Priem;
uint16_t In_X4_Deb(void) //антидребезг SPI2 - SN65HVS881
{
    static uint16_t  cnt_0, cnt_1, cnt_2, cnt_3 ;//, cnt_4 , cnt_5 , cnt_6
    uint16_t delta, sample;
    
    //sample = (RxDI[1]<<8) | RxDI[0] ;
    sample = SPI2_Priem ;
    delta = sample ^ deb_RxDI;
    cnt_3 = (cnt_3 ^ (cnt_2 & cnt_1 & cnt_0)) & delta; //10_05
    cnt_2 = (cnt_2 ^ (cnt_1 & cnt_0)) & delta;
    cnt_1 = (cnt_1 ^ cnt_0) & delta;
    cnt_0 = ~cnt_0 & delta;
    
    deb_RxDI ^= (cnt_0 & cnt_1 & cnt_2);
 
    return deb_RxDI;
}


//////////////////////////расчёт CRC для МАХ14900 - короткий и табличный вариант

  unsigned char Loop_CRC (unsigned char crc, unsigned char byte)
  {
    int i;
    for (i = 0; i < 8; i++)
    {
      crc <<= 1;
      if (crc  & 0x80) crc ^= 0xB7; // 0x37 with MSBit on purpose
      if (byte & 0x80) crc ^= 1;
      byte <<= 1;
    }
    return crc;
  }

 /* unsigned char crcSmallEncode16 (unsigned char byte1, unsigned char byte2)
  {
    unsigned char synd;
    synd = Loop_CRC (0x7f, byte1);
    synd = Loop_CRC (synd, byte2);
    return Loop_CRC (synd, 0x80) | 0x80;
  }  

uint8_t crcSmallCheck16 (unsigned char byte1, unsigned char byte2, unsigned char byte3)
{
	unsigned char synd;

	synd = Loop_CRC (0x7f, byte1);
	synd = Loop_CRC (synd, byte2);
	return Loop_CRC (synd, byte3);//== 0
}*/


//
const unsigned char propagate8_next0 [128] = {
 0x00, 0x6E, 0x6B, 0x05, 0x61, 0x0F, 0x0A, 0x64,
 0x75, 0x1B, 0x1E, 0x70, 0x14, 0x7A, 0x7F, 0x11,
 0x5D, 0x33, 0x36, 0x58, 0x3C, 0x52, 0x57, 0x39,
 0x28, 0x46, 0x43, 0x2D, 0x49, 0x27, 0x22, 0x4C,
 0x0D, 0x63, 0x66, 0x08, 0x6C, 0x02, 0x07, 0x69,
 0x78, 0x16, 0x13, 0x7D, 0x19, 0x77, 0x72, 0x1C,
 0x50, 0x3E, 0x3B, 0x55, 0x31, 0x5F, 0x5A, 0x34,
 0x25, 0x4B, 0x4E, 0x20, 0x44, 0x2A, 0x2F, 0x41,
 0x1A, 0x74, 0x71, 0x1F, 0x7B, 0x15, 0x10, 0x7E,
 0x6F, 0x01, 0x04, 0x6A, 0x0E, 0x60, 0x65, 0x0B,
 0x47, 0x29, 0x2C, 0x42, 0x26, 0x48, 0x4D, 0x23,
 0x32, 0x5C, 0x59, 0x37, 0x53, 0x3D, 0x38, 0x56,
 0x17, 0x79, 0x7C, 0x12, 0x76, 0x18, 0x1D, 0x73,
 0x62, 0x0C, 0x09, 0x67, 0x03, 0x6D, 0x68, 0x06,
 0x4A, 0x24, 0x21, 0x4F, 0x2B, 0x45, 0x40, 0x2E,
 0x3F, 0x51, 0x54, 0x3A, 0x5E, 0x30, 0x35, 0x5B
};

const unsigned char propagate8_next1 [128] = {
 0xB7, 0xD9, 0xDC, 0xB2, 0xD6, 0xB8, 0xBD, 0xD3,
 0xC2, 0xAC, 0xA9, 0xC7, 0xA3, 0xCD, 0xC8, 0xA6,
 0xEA, 0x84, 0x81, 0xEF, 0x8B, 0xE5, 0xE0, 0x8E,
 0x9F, 0xF1, 0xF4, 0x9A, 0xFE, 0x90, 0x95, 0xFB,
 0xBA, 0xD4, 0xD1, 0xBF, 0xDB, 0xB5, 0xB0, 0xDE,
 0xCF, 0xA1, 0xA4, 0xCA, 0xAE, 0xC0, 0xC5, 0xAB,
 0xE7, 0x89, 0x8C, 0xE2, 0x86, 0xE8, 0xED, 0x83,
 0x92, 0xFC, 0xF9, 0x97, 0xF3, 0x9D, 0x98, 0xF6,
 0xAD, 0xC3, 0xC6, 0xA8, 0xCC, 0xA2, 0xA7, 0xC9,
 0xD8, 0xB6, 0xB3, 0xDD, 0xB9, 0xD7, 0xD2, 0xBC,
 0xF0, 0x9E, 0x9B, 0xF5, 0x91, 0xFF, 0xFA, 0x94,
 0x85, 0xEB, 0xEE, 0x80, 0xE4, 0x8A, 0x8F, 0xE1,
 0xA0, 0xCE, 0xCB, 0xA5, 0xC1, 0xAF, 0xAA, 0xC4,
 0xD5, 0xBB, 0xBE, 0xD0, 0xB4, 0xDA, 0xDF, 0xB1,
 0xFD, 0x93, 0x96, 0xF8, 0x9C, 0xF2, 0xF7, 0x99,
 0x88, 0xE6, 0xE3, 0x8D, 0xE9, 0x87, 0x82, 0xEC
};

//
#pragma optimize = none

unsigned char crcFastEncode8 (unsigned char byte1)
{
	unsigned char synd;
	
	synd = (byte1 & 0x80) ? 0xEC : 0x5B; // 6C & 5B before optimization
	synd ^= byte1;
	return propagate8_next1[synd];
}

uint8_t crcFastCheck8 (unsigned char byte1, unsigned char byte2)
{
	unsigned char synd;
	unsigned char const *ptr;

	synd = (byte1 & 0x80) ? 0xEC : 0x5B;
	synd ^= byte1;
	ptr = byte2 & 0x80 ? propagate8_next1 : propagate8_next0;
	return (ptr[synd] ^ byte2) == 0;
}
unsigned char crcFastEncode16 (unsigned char byte1, unsigned char byte2)
{
	unsigned char synd;
	unsigned char const *ptr;

	synd = (byte1 & 0x80) ? 0xEC : 0x5B;
	synd ^= byte1;
	ptr = byte2 & 0x80 ? propagate8_next1 : propagate8_next0;
	synd = ptr[synd] ^ byte2;
	return propagate8_next1[synd];
}


uint8_t crcFastCheck16 (unsigned char byte1, unsigned char byte2, unsigned char byte3)
{
	unsigned char synd;
	unsigned char const *ptr;

	synd = (byte1 & 0x80) ? 0xEC : 0x5B;
	synd ^= byte1;
	ptr = byte2 & 0x80 ? propagate8_next1 : propagate8_next0;
	synd = ptr[synd] ^ byte2;
	ptr = byte3 & 0x80 ? propagate8_next1 : propagate8_next0;
	return (ptr[synd] ^ byte3) ;//== 0
}


/*-------------------------------SPI_1 Init-----------------------------------*/
void SPI_1_Init(void)
{
  	SPI1->CR1 &= (~SPI_CR1_SPE); 
  	SPI1->IFCR |= 0xff ; //очистка всех флагов SPI
  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //SPI1_NSS
    //HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)RxDO, 6);    //Прием Дискретных Выходов
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // X1_Out2 enable
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // X4_Out1 enable
}
  
  
void SPI_1_CRC(void)//
{  	//Out_X4_1 = 1; // test
    TxDO[0] = Out_X1_8 | Out_X1_7 <<1 | Out_X1_6 <<2 | Out_X1_5<<3 | Out_X1_4<<4 | Out_X1_3<<5 | Out_X1_2 <<6 | Out_X1_1 <<7; //0.4мксек
    TxDO[2] = Out_X4_7 <<1 | Out_X4_6 <<2 | Out_X4_5<<3 | Out_X4_4<<4 | Out_X4_3<<5 | Out_X4_2 <<6 | Out_X4_1 <<7 ;//Out_X4_8 0.4мксек
    TxDO[1] = crcFastEncode8 (TxDO[0]); //расчет CRC1  //0.6мксек
	//GPIOA->BSRR = GPIO_BSRR_BR4;//NSS 300нсек
    TxDO[3] = crcFastEncode8 (TxDO[2]); //расчет CRC2  //0.6мксек
}
 
uint16_t tepp = 0;
extern uint8_t Nomer;
uint16_t AnalizSPI[20] = {0};
	 
   #pragma optimize = none
void SPI_1_Rx_Tx(void)
{   
    //f_NSS1 = 1;
    GPIOA->BSRR = GPIO_BSRR_BR4;//NSS
	SPI1->CR1 &= ~SPI_CR1_SPE;		
	//GPIOA->BSRR = GPIO_BSRR_BR4;
    SPI1->CR1 |= SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_CSTART;

	uint8_t TxXferCount = 4, PerN = 0;
	uint16_t t_Out = 2600;
	uint32_t  tickstart = TIM2->CNT; //HAL_GetTick();
	AnalizSPI[0] = tickstart ; // тест скор
	
	while ((TxXferCount > 0UL))
    {
      /// Check the TXP flag //
      if (((SPI1->SR&SPI_FLAG_TXP)!=0) && (TxXferCount > 0UL)) 		//передача 2мксек на 6-ть байт 3,1Мбит
      {																//передача 1,6мксек на 4-ре байта при 6,2Мбит			
        *((__IO uint8_t *)&SPI1->TXDR) = TxDO[PerN]; PerN ++;
        TxXferCount --;
		//AnalizSPI[PerN] = TIM2->CNT - tickstart; // тест скор
      }
	  
	  /// Check the RXP flag //
	  if ((SPI1->SR&SPI_FLAG_RXP)!=0) 
           RxDO[0] = *((__IO uint8_t *)&SPI1->RXDR); 
      
		// Timeout management / - заменить на DWT
	  uint32_t  tick = TIM2->CNT - tickstart;
	  if ((tick >=  t_Out) ){
	  TxXferCount = 0; }
    }
	TIM3->DIER = 1;
	//AnalizSPI[18] = TIM2->CNT - tickstart; 		// тест скор
	
	//SPI1->IFCR = 0xff8; SPI1->IER = 0x0;
	//SPI1->CR1 &= ~SPI_CR1_SPE;

	AnalizSPI[19] = TIM2->CNT - tickstart; 		// на все 17,625 мкс на 6-ть байт 3,1Мбит
	//SPI1->TXDR = TxDO[0];						// на все 7 мкс на 4-ть байт 6,2Мбит	
} 
 
uint8_t Ch_SPI_OpenL=0; //открытая цепь, контроль воткн
uint8_t Ch_SPI_UVLO=0;  //перенапряжение
uint8_t Ch_SPI_Err=0;   //состояние выхода не соответствует
uint8_t Ch_SPI_CRC=0;   //битый пакет приема DO
uint8_t Ch_SPI_NoPow=0; //нет питания Х4
uint8_t Ch_SPI_Termo=0; //перегрев
uint8_t f_SPI_OpenL=0;
uint8_t f_SPI_UVLO=0;  // перенапряжение
uint8_t f_SPI_Err=0; // состояние выхода не соотв заданному
uint8_t f_SPI_CRC=0;
uint8_t f_SPI_NoPow=0; //нет питания Х4
uint8_t f_SPI_Termo=0; //перегрев
uint8_t SPI_OpenL_X1=0; //вывод который не подключен в Х1
uint8_t SPI_OpenL_X4=0; //вывод который не подключен в Х4
uint8_t SPI_UVLO_X1=0;
uint8_t SPI_UVLO_X4=0;
uint8_t SPI_Err_X1=0;
uint8_t SPI_Err_X4=0;
uint8_t Ch_SPI1=0; //счёт не принятых пакетов по SPI1
uint8_t f_SPI1_=0; //ошибка нету связи по SPI1

//проверка и установка флагов ошибок
 void SPI_1_Diagnostic(void)
 {
   if(Ch_SPI1 > 32)
     f_SPI1_ = 1;
   else Ch_SPI1++; //счёт не принятых пакетов по SPI1 //25_01_21 - 
  // if((CRC_1 == RxDO[2])||(CRC_2 == RxDO[5]) ) 	//CRC
   uint8_t l1 = (crcFastEncode8 (RxDO[0]) & 0x7f);	//,RxDO[2]
   uint8_t l2 = (crcFastEncode8 (RxDO[2]) & 0x7f);	//
   if((l1 == RxDO[1]) && (l2 == RxDO[3]))//CRC 
   {
      Ch_SPI_CRC = 0;
      if( (RxDO[0]&0x01 || RxDO[2]&0x01) != 0 )//X1:1 , X1:2 не проверять -&0x3f, X4-?
      {
        Ch_SPI_OpenL++;
        if(Ch_SPI_OpenL > 16)
          f_SPI_OpenL = 1;
        //SPI_OpenL_X1 = RxDO[0]&(~ RxDO[1])&0x3f;
        //SPI_OpenL_X4 = RxDO[3]&(~ RxDO[4]);
      }
      else Ch_SPI_OpenL=0;
   
     /*if( (RxDO[0] & RxDO[1])||(RxDO[3] & RxDO[4])!=0 ) // перенапряжение
      {
        Ch_SPI_UVLO++;
        if(Ch_SPI_UVLO>16)
          f_SPI_UVLO=1;
        //SPI_UVLO_X1=RxDO[0]&RxDO[1];
        //SPI_UVLO_X4=RxDO[3]&RxDO[4];
      }
    else Ch_SPI_UVLO=0; */
     
    if ((TxDO[0] != RxDO[1]) || (TxDO[2] != RxDO[3])) // состояние выхода не соотв заданному
    {
      Ch_SPI_Err++;
      if(Ch_SPI_Err>16)
        f_SPI_Err=1;
      //SPI_Err_X1=TxDO[0]&~RxDO[1];
      //SPI_Err_X4=TxDO[3]&~RxDO[4];
    }
    else Ch_SPI_Err=0;
    
    /*if (RxDO[2] == 0xff)//нет питания Х4
    { 
      Ch_SPI_NoPow++;
      if(Ch_SPI_NoPow > 16) f_SPI_NoPow = 1; 
    }*/
    
    if ((RxDO[0] == 0xff)||(RxDO[2] == 0xff))//перегрев или разъем не подкл
    { 
      Ch_SPI_Termo++;
      if(Ch_SPI_Termo > 16) f_SPI_Termo=1;
    }
    
   }
   /*else if ((((crcFastEncode16 (RxDO[1],RxDO[2])) & 0x7f)==RxDO[3])&&
            (((crcFastEncode16 (RxDO[4],RxDO[5])) & 0x7f)==RxDO[0]))
   {
     Ch_SPI_CRC=0;
     if( (RxDO[1]&0x01||RxDO[4]&0x01)!=0 )//X1:1 , X1:2 не проверять -&0x3f, X4-?
      {
        Ch_SPI_OpenL++;
        if(Ch_SPI_OpenL>16)
          f_SPI_OpenL = 1;
      }
     else Ch_SPI_OpenL=0;
   
     if( (RxDO[1] & RxDO[2])||(RxDO[4] & RxDO[5])!=0 ) // перенапряжение
      {
        Ch_SPI_UVLO++;
        if(Ch_SPI_UVLO>16)
          f_SPI_UVLO=1;
      }
    else Ch_SPI_UVLO=0; 
     
    if ((TxDO[1] != RxDO[2]) || (TxDO[4] != RxDO[5])) // состояние выхода не соотв заданному
    {
      Ch_SPI_Err++;
      if(Ch_SPI_Err>16)
        f_SPI_Err=1;
    }
    else Ch_SPI_Err=0;
    
    if (RxDO[4]&RxDO[5]==0xff)//нет питания Х4
    { 
      Ch_SPI_NoPow++;
      if(Ch_SPI_NoPow>16)f_SPI_NoPow=1; 
    }
    
    if ((RxDO[1]==0xff)||(RxDO[4]==0xff))//перегрев или разъем не подкл
    { 
      Ch_SPI_Termo++;
      if(Ch_SPI_Termo>16)f_SPI_Termo=1;
    }
    
   }*/
   else 
   {
     Ch_SPI_CRC++;
     if(Ch_SPI_CRC > 16) //
       f_SPI_CRC=1;
   }
     
   //if ((f_SPI_OpenL||f_SPI_UVLO||f_SPI_Err)!=0 )f_SPI1_=1; //TEMP HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)проверка флагов  и светодиод
   f_SPI1_ |= (f_SPI_OpenL||f_SPI_UVLO||f_SPI_Err);
   f_SPI_OpenL=0; f_SPI_UVLO=0; f_SPI_Err=0;
 }


 
/*-------------------------------SPI_2 -----------------------------------*/
uint8_t Ch_SPI2_UVO=0;  //перенапряжение
uint8_t Ch_SPI2_HOT=0;  //перегрев
uint8_t Ch_SPI2_PAR=0;
uint8_t f_SPI2_UVO=0;  //перенапряжение на входах  Х4
uint8_t f_SPI2_HOT=0;  //перегрев микросхемы входов Х4
uint8_t f_SPI2_PAR=0;  //ошибка проверки бита четности
uint8_t Ch_SPI2=0;     //счётчик не принятых пакетов по SPI2
uint8_t f_SPI2_=0;     //ошибка нету связи по SPI2
//uint8_t F_SPI2_Dis=0;					 
 
void SPI_2_Init(void) // на вх. упр-я  нули
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);        //nss2-0
  GPIOB->BSRR = GPIO_BSRR_BS15;//LD-1
  
} 
  
  #pragma optimize = none
 void SPI_2_Rx_Tx(void)  ///0,8мкс
{
  uint8_t n1=0; //SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);      
  GPIOB->BSRR = GPIO_BSRR_BR15;//LD-0
  while(n1 < 21) n1++; n1=0;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);        
  GPIOB->BSRR = GPIO_BSRR_BS15;//LD-1
  while(n1 < 11) n1++; n1=0;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);      
  GPIOB->BSRR = GPIO_BSRR_BR12;//nss2-0
  
  //Прием Дискретных Входов
  //HAL_SPI_Receive(&hspi2, (uint8_t*)RxDI, 2, 10); //if(HAL_SPI_Receive(&hspi2, (uint8_t*)&RxDI[0], 2, 10) != HAL_OK) 
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);        //nss2-1
  //Ch_SPI2 = 0;
  
   //__HAL_LOCK(&hspi2); 
  hspi2.Lock = HAL_LOCKED;
  // Enable SPI peripheral /
  __HAL_SPI_ENABLE(&hspi2);
  __HAL_SPI_ENABLE_IT(&hspi2,SPI_IT_RXNE);
  SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
  //} //13_03_20  
}


       
void SPI_2_Diagnostic(void)
{
  Ch_SPI2++; //счёт не принятых пакетов по SPI2
  if(Ch_SPI2>32)f_SPI2_=1;
  
  int x=0;
  x= SPI2_Priem ^(SPI2_Priem>>8); //бит четности
  x^=x>>4;
  x^=x>>2;
  x^=x>>1;
  //

  if(x&1)
  {
      Ch_SPI2_PAR=0;
      if( (SPI2_Priem & 0x40) == 0 )//перенапряжение
      {   SPI2_Priem =0; //19_03_20
          Ch_SPI2_UVO++;
          if(Ch_SPI2_UVO>16)f_SPI2_UVO=1;
      }
      else Ch_SPI2_UVO=0;
     
      if( (SPI2_Priem & 0x80) ==0 )//перегрев
      {   
          SPI2_Priem =0; //19_03_20
          Ch_SPI2_HOT++;
          if(Ch_SPI2_HOT>16)f_SPI2_HOT=1;
      }
      else Ch_SPI2_HOT=0;
  }
  else
  {   
      SPI2_Priem =0; //19_03_20
      Ch_SPI2_PAR++;
      if(Ch_SPI2_PAR>16)f_SPI2_PAR=1; //ошибка проверки четности 
  }
  
  f_SPI2_ |= (f_SPI2_PAR || f_SPI2_HOT || f_SPI2_UVO);
  //if((Ch_SPI2 == 2)||(Ch_SPI2_UVO == 2)||(Ch_SPI2_HOT == 2)){ //||(f_SPI2_)13_03_20
  //  __HAL_SPI_DISABLE(&hspi2); //13_03_20
  //F_SPI2_Dis^=1;} //13_03_20_
  f_SPI2_PAR=0; f_SPI2_HOT=0; f_SPI2_UVO=0;
}

//

