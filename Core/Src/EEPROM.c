
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "EEPROM.h"
#include "UART1.h" //crc8
#include "Modbus.h"
#include "Lib_SROM.h"

/* ------ variables ---------------------------------------------------------*/

 extern I2C_HandleTypeDef hi2c4;
 //extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
 extern  USHORT   usRegInputBuf[REG_INPUT_NREGS]; 

 uint8_t I2C_TxBuff[32]={0}; //temp
 uint8_t I2C_RxBuff[32]={0}; //temp
 uint8_t RAM_BackUp[33]={0}; //temp
 uint8_t  F_I2C_Wr=0,   F_I2C_Rd=0;
 uint8_t  F2_I2C_Wr=0,   F2_I2C_Rd=0;
 //uint8_t  F2_I2C_Wr=0,  F2_I2C_Rd=0;
 uint16_t  Ch_I2C_Wr=0;
 //extern uint8_t  Io_TA1;
 //extern uint8_t  Io_TA2;
 extern uint8_t  V1_Koeff;
 extern uint8_t  V2_Koeff;
 extern uint8_t  V3_Koeff;
 extern uint16_t LTD_Threshold;
 //extern uint8_t  K_V1;
 //extern uint8_t  K_V2;
 //extern uint8_t  K_V3;
 //03_09
 extern uint32_t FAULT_V;
 extern uint32_t FAULT_O;
 //extern int16_t I1_Otkl;
 //extern int16_t I2_Otkl;
 //extern int16_t I3_Otkl;
 //extern int16_t I4_Otkl;
 //extern int32_t II_Otkl;
 extern uint16_t LTD_Izmer;
 //extern uint8_t Buff_Sobitij [ ];
 extern uint8_t i_BS;
 extern uint32_t FAULT;
 extern uint16_t Ch_I2C;
 uint8_t F_EE_Prov=0;
 //extern uint8_t STATUS;
 uint8_t f_Rd_I2C = 0;

 // Таблица CRC
 extern const uint8_t Crc8Table[256];
 /*
I2C4
		PF11  - (WP) 
		PF14  - (SCL)
		PF15  - (SDA)
*/

 
/* ------ functions ---------------------------------------------------------*/

void Zapis_EEPROM (uint8_t AdrStra) //запись в EEPROM 24LC64T , после записи задержка не менее 5-ти мс 
{   //Ch_I2C =0;  //06_05_20 
    HAL_GPIO_WritePin (GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);   // I2C - WP=0 разрешение записи
    Ch_I2C_Wr ++;
    if (Ch_I2C_Wr == 30) //26_03_20 повысил с 20 до 30
    {
      if(AdrStra == 0x01){
		uint8_t P_Io_Set_H = P_Io_Set>>8, P_Io_Set_L = P_Io_Set, N_Io_Set_H = N_Io_Set>>8, N_Io_Set_L = N_Io_Set;
		uint8_t P_Imax_Set_H = P_Imax_Set>>8, P_Imax_Set_L = P_Imax_Set, N_Imax_Set_H = N_Imax_Set>>8, N_Imax_Set_L = N_Imax_Set;
		uint8_t P_dIdt_Set_H = P_dIdt_Set>>8, P_dIdt_Set_L = P_dIdt_Set, N_dIdt_Set_H = N_dIdt_Set>>8, N_dIdt_Set_L = N_dIdt_Set;
		uint8_t LTD_Thr_H = LTD_Threshold>>8, LTD_Thr_L = LTD_Threshold, Count_Switch_H =Count_Switch>>8, Count_Switch_L = Count_Switch; 
		I2C_TxBuff[0]=P_Io_Set_L;         I2C_TxBuff[1]=P_Io_Set_H; 
		I2C_TxBuff[2]=N_Io_Set_L;         I2C_TxBuff[3]=N_Io_Set_H; 
		I2C_TxBuff[4]=P_Imax_Set_L;       I2C_TxBuff[5]=P_Imax_Set_H; 
		I2C_TxBuff[6]=P_Imax_TSet;  
		I2C_TxBuff[7]=N_Imax_Set_L;       I2C_TxBuff[8]=N_Imax_Set_H; 
		I2C_TxBuff[9]=N_Imax_TSet; 
		I2C_TxBuff[10]=P_dIdt_Set_L;      I2C_TxBuff[11]=P_dIdt_Set_H; 
		//I2C_TxBuff[12]=P_dIdt_TSet; 	//убираем
		I2C_TxBuff[13]=N_dIdt_Set_L;      I2C_TxBuff[14]=N_dIdt_Set_H; 
		//I2C_TxBuff[15]=N_dIdt_TSet;     //убираем
		I2C_TxBuff[16]=LTD_Thr_L;         I2C_TxBuff[17]=LTD_Thr_H; 
		I2C_TxBuff[18]=Count_Switch_L;    I2C_TxBuff[19]=Count_Switch_H; 
		I2C_TxBuff[20]= Io_TA1;           I2C_TxBuff[21]= Io_TA2;
		I2C_TxBuff[22]= K_V1;      		  I2C_TxBuff[23]= K_V2;      
		//I2C_TxBuff[24]= K_V3; //убираем
		I2C_TxBuff[25]= Trig_Prot;
		I2C_TxBuff[26]= CT1_QS1; 
		I2C_TxBuff[27]= STATUS; //25_08_20 
		I2C_TxBuff[28]= Crc8(I2C_TxBuff, 28);
		HAL_I2C_Mem_Write_IT(&hi2c4, (uint16_t)0xa0, AdrStra, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_TxBuff, 29);
      }
      
      if(AdrStra == 0x02){ //25.02.25
		I2C_TxBuff[0] = N_AFB1;         I2C_TxBuff[1] = ((N_AFB1) >> 8); 	//86
		I2C_TxBuff[2] = N_AFB2;     														//87
		I2C_TxBuff[3] = N_AFB3;        	I2C_TxBuff[4] = ((N_AFB3) >> 8); 	//88
		I2C_TxBuff[5] = N_MUA1;         I2C_TxBuff[6] = ((N_MUA1) >> 8); 	//89
		I2C_TxBuff[7] = N_MUA2;     														//90
		I2C_TxBuff[8] = N_MUA3;        	I2C_TxBuff[9] = ((N_MUA3) >> 8);	//91
		I2C_TxBuff[10] = Crc8(I2C_TxBuff, 10);
		HAL_I2C_Mem_Write_IT(&hi2c4, (uint16_t)0xa0, 0x21, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_TxBuff, 11);
      }
    }
    
    if (Ch_I2C_Wr >= 240) //12mcek
    {
      F_I2C_Wr = 0; 
      F_I2C_Rd = AdrStra; 
      F_EE_Prov ++; //30_01_20 для проверки сохранения
      Ch_I2C_Wr = 0;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);   // I2C - WP=1 защита от записи */
    }

}

////////////////////////////////////////////////////////////////////////////////
 uint16_t  Ch_I2C_Rd = 0;
 uint8_t Fault_27 = 0;
 extern  uint8_t F2_I2C_Rd;
 extern  uint8_t Faultec ;

void Chtenie_EEPROM (uint8_t AdrStra) //50 mks
{     //Ch_I2C =0; //06_05_20      
  
    if ( Ch_I2C_Rd == 10){  							//CopyBackUpinData( 28 );
	  if(AdrStra == 0x01) HAL_I2C_Mem_Read_IT(&hi2c4, (uint16_t)0xa0, 0x01, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_RxBuff, 29);
	  if(AdrStra == 0x02) HAL_I2C_Mem_Read_IT(&hi2c4, (uint16_t)0xa0, 0x21, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_RxBuff, 11);
    }
    Ch_I2C_Rd ++;
    
    if ( Ch_I2C_Rd > 120) //6мсек
    {
        uint8_t s;
	
        if(AdrStra == 0x01) 
		{
		  s = Crc8(I2C_RxBuff, 28); 
		
		  if(F_EE_Prov != 0) //30_01_20 для проверки сохранения
		  { 
			  Ch_I2C_Rd = 0;
			  F_I2C_Rd = 0; 
			  if(s==I2C_RxBuff[28]) F_EE_Prov = 0; //if(s==I2C_TxBuff[27]) F_EE_Prov=0;
			  else 
			  { 
				if(F_EE_Prov < 4) F_I2C_Wr = 1;
				else FAULT |= 0x04000000; //Fault_27
			  }
		  }
		  else 
		  {
			  if(s == I2C_RxBuff[28]) //если CRC верно
			  {
			  P_Io_Set = I2C_RxBuff[0] | I2C_RxBuff[1] << 8;
			  N_Io_Set = I2C_RxBuff[2] | I2C_RxBuff[3] << 8;
			  P_Imax_Set = I2C_RxBuff[4] | I2C_RxBuff[5] << 8;
			  P_Imax_TSet = I2C_RxBuff[6] ;
			  N_Imax_Set = I2C_RxBuff[7] | I2C_RxBuff[8] << 8;
			  N_Imax_TSet = I2C_RxBuff[9] ;
			  P_dIdt_Set = I2C_RxBuff[10] | I2C_RxBuff[11] << 8;
			  //P_dIdt_TSet = I2C_RxBuff[12] ;  //убираем
			  N_dIdt_Set = I2C_RxBuff[13] | I2C_RxBuff[14] << 8;
			  //N_dIdt_TSet = I2C_RxBuff[15] ;  //убираем
			  LTD_Threshold = I2C_RxBuff[16] | I2C_RxBuff[17] << 8;
			  //LTD_Thresh = LTD_Threshold * 10; // 18_10_19  //убираем
			  Count_Switch = I2C_RxBuff[18] | I2C_RxBuff[19] << 8; // TEMP 108;//
			  F_I2C_Rd = 0;
			  Io_TA1 = I2C_RxBuff[20];
			  Io_TA2 = I2C_RxBuff[21];
			  K_V1 = (int8_t)I2C_RxBuff[22]; //22_11 залить потом
			  K_V2 = (int8_t)I2C_RxBuff[23]; //22_11
			  //K_V3 = (int8_t)I2C_RxBuff[24]; //22_11  //убираем
			  V1_Koeff= (568 *(1000+(int8_t)I2C_RxBuff[22]))/1000;
			  V2_Koeff= (568 *(1000+(int8_t)I2C_RxBuff[23]))/1000;
			  V3_Koeff= (568 *(1000+(int8_t)I2C_RxBuff[24]))/1000;
			  Trig_Prot = I2C_RxBuff[25];
			  CT1_QS1 = I2C_RxBuff[26]; //06_06
			  STATUS = I2C_RxBuff[27]; //25_08_20 
			  f_Rd_I2C = 0;
			  //F2_I2C_Rd=1;//чтение значений из EEPROM 2
			  }
			  else 
			  {
			  f_Rd_I2C++; 
			  if(f_Rd_I2C > 3){
				Fault_27 = 1;  Faultec = 1;
				//установка значений по умолчанию 27_02
				P_Io_Set = 3000;           N_Io_Set = 1000;
				P_Imax_Set = 2500;         N_Imax_Set = 800;
				P_Imax_TSet = 50 ;         N_Imax_TSet = 50 ;
				P_dIdt_Set = 2000;         N_dIdt_Set = 1000;
				//P_dIdt_TSet = 1 ;          N_dIdt_TSet = 1 ;  //убираем
				//LTD_Threshold = 1000;
				F_I2C_Wr = 1;         F_I2C_Rd = 0;	f_Rd_I2C = 0;
				Trig_Prot = 0xfe;}
			  }
		  }
		}
      
      if(AdrStra == 0x02) //
	  {
		  s = Crc8(I2C_RxBuff, 10); 
		
		  if(F_EE_Prov != 0) //30_01_20 для проверки сохранения
		  { 
			Ch_I2C_Rd = 0;
			F_I2C_Rd = 0;  f_Rd_I2C = 0;
			if(s == I2C_RxBuff[10]) F_EE_Prov = 0; //if(s==I2C_TxBuff[27]) F_EE_Prov=0;
			else 
			{ 
				if(F_EE_Prov < 4) F_I2C_Wr = 2;
				else FAULT |= 0x04000000; //Fault_27
			}
		  }
		  else 
		  {
			if(s == I2C_RxBuff[10]) //если CRC верно
			{  //по новому 4...6 8...10
				N_AFB1 = I2C_RxBuff[0] | I2C_RxBuff[1] << 8;
				N_AFB2 = I2C_RxBuff[2] ;
				N_AFB3 = I2C_RxBuff[3] | I2C_RxBuff[4] << 8;
				N_MUA1 = I2C_RxBuff[5] | I2C_RxBuff[6] << 8;
				N_MUA2 = I2C_RxBuff[7] ;
				N_MUA3 = I2C_RxBuff[8] | I2C_RxBuff[9] << 8;
				F_I2C_Rd = 0;
				f_Rd_I2C = 0;
			}
			else 
			{
				f_Rd_I2C++; 
				if(f_Rd_I2C > 3) { 
				  Fault_27 = 1;  Faultec = 1; //установка значений по умолчанию ?
				  //F_I2C_Wr = 1;            
				  F_I2C_Rd = 0; f_Rd_I2C = 0;
				} //Trig_Prot = 0xfe;
			}
		  }
	  }
      F_I2C_Rd = 0;  Ch_I2C_Rd = 0;
    }
}


////////////////////////////////////////////////////////////////////////////////

//int16_t I1_Otkl=0;
extern uint8_t Language;

void Zapis_EEPROM_2(void) ////запись в EEPROM 24LC64T , после записи задержка не менее 5-ти мс 
{
  //if ((F_I2C_Wr==1)&&(F_I2C_Rd==0)){//перенёс 
  //Ch_I2C =0;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);   // I2C - WP=0 разрешение записи
    //Ch_I2C_Wr ++;
    //if (Ch_I2C_Wr == 20)
    //{
      uint8_t I1_Otkl_H = I1_Otkl>>8, I1_Otkl_L = I1_Otkl, I2_Otkl_H = I2_Otkl>>8, I2_Otkl_L = I2_Otkl;
      uint8_t I3_Otkl_H = I3_Otkl>>8, I3_Otkl_L = I3_Otkl, I4_Otkl_H = I4_Otkl>>8, I4_Otkl_L = I4_Otkl;
      uint8_t LTD_Izmer_H = LTD_Izmer>>8, LTD_Izmer_L = LTD_Izmer;
      uint8_t FAULT_4 = FAULT_V>>24, FAULT_3 = FAULT_V>>16, FAULT_2 = FAULT_V>>8, FAULT_1 = FAULT_V;
      uint8_t II_Otkl_H = II_Otkl>>16, II_Otkl_M = II_Otkl>>8, II_Otkl_L = II_Otkl; //17_12_19
      uint8_t FAULT_8 = FAULT_O>>24, FAULT_7 = FAULT_O>>16, FAULT_6 = FAULT_O>>8, FAULT_5 = FAULT_O;

      RAM_BackUp[0]=I1_Otkl_L;       RAM_BackUp[1]=I1_Otkl_H; // токи
      RAM_BackUp[2]=I2_Otkl_L;       RAM_BackUp[3]=I2_Otkl_H; 
      RAM_BackUp[4]=I3_Otkl_L;       RAM_BackUp[5]=I3_Otkl_H;
      RAM_BackUp[6]=I4_Otkl_L;       RAM_BackUp[7]=I4_Otkl_H; 
      RAM_BackUp[8]=LTD_Izmer_L;     RAM_BackUp[9]=LTD_Izmer_H;
      RAM_BackUp[10]=FAULT_1;        RAM_BackUp[11]=FAULT_2; //тек. ошибки
      RAM_BackUp[12]=FAULT_3;        RAM_BackUp[13]=FAULT_4;
      RAM_BackUp[14]=II_Otkl_L;      RAM_BackUp[15]=II_Otkl_M;    RAM_BackUp[16]=II_Otkl_H; //ток TA2
      uint8_t n=0;
      while (n < 9) // сохранение буффера событий 10значений
      {
        if ((i_BS -n)>=0) 
        { RAM_BackUp[17+n]=usRegInputBuf [i_BS-n];}
        else 
        { RAM_BackUp[17+n]=usRegInputBuf [i_BS-n+100];}
        n++;
      }
      RAM_BackUp[27]=Language; 
      RAM_BackUp[28]=FAULT_5; //21_12_19
      RAM_BackUp[29]=FAULT_6;
      RAM_BackUp[30]=usRegHoldingBuf[86]; //26_03_24
      RAM_BackUp[31]=usRegHoldingBuf[87]; //26_03_24
      RAM_BackUp[32]=i_BS; 
     
      //HAL_I2C_Mem_Write_IT(&hi2c3, (uint16_t)0xa0, 0x41, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_TxBuff, 32);
      CopyDatainBackUp( 33 , 0 ); //11_03_20
      
      // Modbass - 
        //usRegHoldingBuf[52]=I1_Otkl;    usRegHoldingBuf[53]=I2_Otkl; //
        //usRegHoldingBuf[54]=I3_Otkl;    usRegHoldingBuf[55]=I4_Otkl;
        usRegHoldingBuf[56]=LTD_Izmer; 
        usRegHoldingBuf[57]=FAULT_V;    usRegHoldingBuf[58]=FAULT_V>>16;
        //usRegHoldingBuf[59]=II_Otkl;    usRegHoldingBuf[60]=II_Otkl>>16; //17_12_19
		
        usRegHoldingBuf[65]=RAM_BackUp[16]; 
        usRegHoldingBuf[66]=RAM_BackUp[17]; 
        usRegHoldingBuf[67]=RAM_BackUp[18]; usRegHoldingBuf[68]=RAM_BackUp[19]; 
        usRegHoldingBuf[69]=RAM_BackUp[20]; usRegHoldingBuf[70]=RAM_BackUp[21];
        usRegHoldingBuf[71]=RAM_BackUp[22]; usRegHoldingBuf[72]=RAM_BackUp[23];
        usRegHoldingBuf[73]=RAM_BackUp[24]; usRegHoldingBuf[74]=RAM_BackUp[25];
        usRegHoldingBuf[75]=FAULT_O; usRegHoldingBuf[76]=FAULT_O>>16;
        usRegHoldingBuf[77]=i_BS;
	
        DatainBackUp( );
   // }
    
  //  if (Ch_I2C_Wr >= 250)
  //  {
   //   F2_I2C_Wr=0;
    //  Ch_I2C_Wr=0;
   //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // I2C - WP=1 защита от записи 
    //}
  //}
}




void Chtenie_EEPROM_2(void) // после включения
{       
      //if (F_I2C_Rd!=0 && F_I2C_Wr==0){ //Ch_I2C =0;
      //if ( Ch_I2C_Rd==10) CopyBackUpinData( 32 );
      //HAL_I2C_Mem_Read_IT(&hi2c3, (uint16_t)0xa0, 0x41, I2C_MEMADD_SIZE_16BIT, (uint8_t*)I2C_RxBuff, 32);
     // Ch_I2C_Rd ++;
     // if ( Ch_I2C_Rd>140){
    CopyBackUpinData( 32 , 0 );
        I1_Otkl = (RAM_BackUp[1]<<8)|RAM_BackUp[0];
        I2_Otkl = (RAM_BackUp[3]<<8)|RAM_BackUp[2];
        I3_Otkl = (RAM_BackUp[5]<<8)|RAM_BackUp[4];
        I4_Otkl = (RAM_BackUp[7]<<8)|RAM_BackUp[6];
        
        LTD_Izmer = (RAM_BackUp[9]<<8)|RAM_BackUp[8];
        FAULT_V = (RAM_BackUp[13]<<24)|(RAM_BackUp[12]<<16)|(RAM_BackUp[11]<<8)|RAM_BackUp[10];//ошибки при откл-е
        II_Otkl = (RAM_BackUp[16]<<16)|(RAM_BackUp[15]<<8)|RAM_BackUp[14];//ток TA2 при откл-е (0xff<<24)|
        if (RAM_BackUp[16] & 0x80) II_Otkl |= (0xff<<24);
        //Language= I2C_RxBuff[24];
        FAULT_O = (RAM_BackUp[31]<<24)|(RAM_BackUp[30]<<16)|(RAM_BackUp[29]<<8)|RAM_BackUp[28];//ошибки после откл-е
        i_BS = RAM_BackUp[32];
        // Modbass 
        //usRegHoldingBuf[52]=I1_Otkl;    usRegHoldingBuf[53]=I2_Otkl; //
        //usRegHoldingBuf[54]=I3_Otkl;    usRegHoldingBuf[55]=I4_Otkl;
        usRegHoldingBuf[56]=LTD_Izmer; 
        usRegHoldingBuf[57]=FAULT_V;    usRegHoldingBuf[58]=FAULT_V>>16;
        usRegHoldingBuf[59]=II_Otkl;    usRegHoldingBuf[60]=II_Otkl>>16; //17_12_19
        usRegHoldingBuf[65]=RAM_BackUp[16]; 
        usRegHoldingBuf[66]=RAM_BackUp[17]; 
        usRegHoldingBuf[67]=RAM_BackUp[18]; usRegHoldingBuf[68]=RAM_BackUp[19]; 
        usRegHoldingBuf[69]=RAM_BackUp[20]; usRegHoldingBuf[70]=RAM_BackUp[21];
        usRegHoldingBuf[71]=RAM_BackUp[22]; usRegHoldingBuf[72]=RAM_BackUp[23];
        usRegHoldingBuf[73]=RAM_BackUp[24]; usRegHoldingBuf[74]=RAM_BackUp[25];
        usRegHoldingBuf[75]=FAULT_O; usRegHoldingBuf[76]=FAULT_O>>16;
        usRegHoldingBuf[77]=i_BS;
		usRegHoldingBuf[86] = RAM_BackUp[30]; usRegHoldingBuf[87] = RAM_BackUp[31]; //26_03_24
        F2_I2C_Rd =0; //чтение значений из EEPROM 2
        Ch_I2C_Rd =0; BackUpinData( );
     // }
    //}
}


///////////////// SROM /////////////////// SROM ////////////////// SROM ///////////////////

/*void EnableBKUPmem(void)*/

void CopyDatainBackUp(uint8_t Size, uint8_t St_Adr)
{
  uint8_t i = St_Adr;
  // *(__IO uint32_t *) CR2_DBP_BB = (uint32_t)ENABLE; //Disable backup domain write protection EnableBKUPmem();
  SET_BIT (PWR->CR1, PWR_CR1_DBP);
  while (i < Size){
    *(__IO uint32_t *) (D3_BKPSRAM_BASE +i) = RAM_BackUp [i]; i++; //if(i > 102)i++;
  }
  // *(__IO uint32_t *) CR1_DBP_BB = (uint32_t)DISABLE; //Enable backup domain write protection DysableBKUPmem();
  CLEAR_BIT (PWR->CR1, PWR_CR1_DBP);
}

void CopyBackUpinData(uint8_t Size, uint8_t St_Adr)
{
  uint8_t i=St_Adr;
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)ENABLE; //Disable backup domain write protection EnableBKUPmem();
  HAL_PWR_EnableBkUpAccess();
  while (i < Size){
      RAM_BackUp [i] = *(__IO uint32_t *) (D3_BKPSRAM_BASE +i); i++; //if(i > 102)i++;
  }
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)DISABLE; //Enable backup domain write protection DysableBKUPmem();
  HAL_PWR_DisableBkUpAccess();
}

void DatainBackUp(void) //буфер соб-й - в бекап
{
  uint8_t i = 0;
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)ENABLE; //Disable backup domain write protection EnableBKUPmem();
  HAL_PWR_EnableBkUpAccess();
  while (i < 100){
    *(__IO uint32_t *) (D3_BKPSRAM_BASE +i +33) = usRegInputBuf [i]; i++; //if(i > 102)i++;
  }
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)DISABLE; //Enable backup domain write protection DysableBKUPmem();
  HAL_PWR_DisableBkUpAccess();
}

void BackUpinData(void) //бекап - в буфер соб-й 
{
  uint8_t i = 0;
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)ENABLE; //Disable backup domain write protection EnableBKUPmem();
  HAL_PWR_EnableBkUpAccess();
  while (i < 100){
      usRegInputBuf [i] = *(__IO uint32_t *) (D3_BKPSRAM_BASE +i +33); i++; //if(i > 102)i++;
  }
  //*(__IO uint32_t *) CR_DBP_BB = (uint32_t)DISABLE; //Enable backup domain write protection DysableBKUPmem();
  HAL_PWR_DisableBkUpAccess();
}


