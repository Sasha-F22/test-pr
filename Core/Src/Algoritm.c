
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "Algoritm.h"
//#include "Tiristor.h"
#include "stdio.h"
#include "Modbus.h"

/* Variables -----------------------------------------------------------------*/
uint16_t Buff_Sobitij [100]={0}; //Буффер событий
uint8_t i_BS=0;
uint16_t f_Zapr_Zar = 0;

uint8_t Fault_02=0, Fault_03=0, Fault_07=0, Fault_08=0, Fault_09=0, Fault_10=0;
uint8_t Fault_11=0, Fault_12=0, Fault_15=0, Fault_19=0, Fault_20=0, Fault_21=0;
uint8_t Fault_23=0, Fault_24=0, Fault_25=0, Fault_31=0, Fault_30=0;
extern uint8_t Fault_26;
extern uint8_t Fault_13;


//extern int16_t I1_N_1_6ms; // 1.6мс или 32 измерений через 0,05мс каждое 
extern int16_t I1_N_64ms;// 64мс или 128 измерений через 0,5мс каждое
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

//Состояние QF1
#define inp1_3 ((Input_X1 & 0x0004)>>2) //БВ отключен
#define inp1_4 ((Input_X1 & 0x0008)>>3) //БВ включен
#define inp1_6 ((Input_X1 & 0x0020)>>5) //КМ2 включен

extern uint8_t Out_X4_1, Out_X4_2, Out_X4_3, Out_X4_4, Out_X4_5, Out_X4_6, Out_X4_7;
extern uint8_t Out_X1_3;     //Управление QF1 - Включить
extern uint8_t Out_X1_6;     //Управление Вентилятор - Включить

/*-----------------------Управление разъединителем QS1------------------------*/
#define TT1_QS1 580     //Заданный интервал в циклах для контроля времени включения (отключения)
#define TT3_QS1 700    //Заданный интервал в циклах для контроля времени неопределённого состояния было 

uint8_t F0_QS1 = 0;     //Состояние QS1: 0 - отключен 1 - включен 2 - неопределенное состояние
uint8_t F1_QS1 = 0;     //Флаг управления циклом включения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F2_QS1 = 0;     //Флаг управления циклом отключения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F8_QS1 = 0;     //Внешнее управление (команда): 0 - нет команды 1 - включить 2 - отключить
uint8_t F9_QS1 = 0;     //Копия F8_QS1 (для цепочных команд)
uint8_t f_QS_f12 = 0;
uint8_t Ch_QS_f12 = 0;

uint16_t T1_QS1 = 0;    //Счетчик циклов при включении
uint16_t T2_QS1 = 0;    //Счетчик циклов при отключении
uint16_t T3_QS1 = 0;    //Счетчик циклов при неопределённом состоянии

//extern uint16_t CT1_QS1;       //Счетчик включений/выкл-й

//Состояние QS1
#define inp1_1 (Input_X1 & 0x0001)
#define inp1_2 ((Input_X1 & 0x0002)>>1)

extern uint8_t Out_X1_1, Out_X1_2;      //Управление QS1
extern uint32_t FAULT;
extern uint8_t F8_KM1;
//extern uint16_t Input_X1;
//extern uint16_t Input_X4;
extern uint8_t F2_KQF1;     //Флаг упр-я циклом откл-я: 0 - не запущен 1 - выполняется 2 - завершен
uint8_t Faultec = 0;
extern __IO uint16_t SPI2_Priem ;
uint8_t Ch_Faiult2 = 0;


void QS1_Control(void)  //10 ms
{
  static uint8_t Z;
  //письмо 15_03_21
  
  if(f_QS_f12){Ch_QS_f12 ++; if(Ch_QS_f12 > 150){f_QS_f12 = 0; Ch_QS_f12 =0; 
  F8_QS1 =2;}} //задержка откл-я QS1 по 12-й ошибке
 
  
  F9_QS1 = F8_QS1;
  if ((F9_QS1==1)&&(F1_QS1==0)&&(F2_QS1==0)&&((FAULT & 0x10000)==0)){ //16_07_20  + &&((FAULT & 0x10000)==0)
    F1_QS1=1;
    Out_X1_1=1;
    Out_X1_2=0;
    T1_QS1 = TT1_QS1;
    T2_QS1=0;
    T3_QS1=0; //08_10_20
  }
	
  if (F1_QS1==1){
    if ((inp1_1==0)&&(inp1_2==1)){ //иногда нехват-т длит-ти сигнала вкл.
      Z--;
      if (Z==0){
        Out_X1_1=0; //добавить задержку
        Out_X1_2=0;
        T1_QS1=0;
        F1_QS1=0;
        Z=2;// printf("Разъединитель QS1 включен \n");  
      }
    }
    else {
      if (T1_QS1!=0){ T1_QS1--; }
      else { 
        Out_X1_1=0;
        Out_X1_2=0;
        F1_QS1=0;
        Fault_07=1; //printf("Разъединитель QS1 неисправен \n");  //
        Faultec = 1;
        F9_QS1 = 2;
        F8_KM1 = 2; //31_01_20
      }
    }
  }
  
  if ((F9_QS1==2)&&(F2_QS1==0)&&(((FAULT & 0x280000)==0))){ // ||((FAULT & 0x000001)==0)&&(F1_QS1==0) //31_01_20 //&&((((int16_t)I1_N_1_6ms > 100)||((int16_t)I1_N_1_6ms < (-100)))==0)
    F1_QS1=0; // 14_05 TEMP
    F2_QS1=1;
    Out_X1_1=0;
    Out_X1_2=1;
    T2_QS1 = TT1_QS1;
    T1_QS1=0;
    T3_QS1=0; //08_10_20
  }
	
  if (F2_QS1!=0) { 
    if ((inp1_1==1)&&(inp1_2==0)) {
      T2_QS1=0;//printf("Разъединитель QS1 отключен \n");
      F2_QS1=0;
      Out_X1_1=0;
      Out_X1_2=0;
      CT1_QS1++;
    }
    else {
      if (T2_QS1!=0) { T2_QS1--; }
      else {
        Out_X1_1=0;
        Out_X1_2=0;
        F2_QS1=0;
        Fault_07=1; //printf("Разъединитель QS1 неисправен \n");
        Faultec = 1;
      }
    }
  }

  if (inp1_1==1) {
    if (inp1_2==0) {
        F0_QS1=0; //T3_QS1=0; //22_01_21
    }
    else {
        F0_QS1=2; //T3_QS1++; //22_01_21
    }
  }
  else {
    if (inp1_2==1) {
        F0_QS1=1; //T3_QS1=0; //22_01_21
    }
    else {
        F0_QS1=2; //T3_QS1++; //22_01_21
    }
  }
  
  F8_QS1=0;
  F9_QS1=0;
  
  if (F0_QS1 == 2) 
  {//printf("Разъединитель QS1 неисправен \n");
    if(T3_QS1 > TT3_QS1)FAULT |= 0x0200; //{Fault_10=1; Faultec = 1;}//Контроль сигнала положения QS1(оба сигнала одинаковы)
    else T3_QS1++;
  } 
  
  if((F2_QS1 == 0)&&(F1_QS1 == 0)&&(F0_QS1 == 2)) { //22_03_24 - Ш захотел добавить
    if(Ch_Faiult2 > 10) FAULT |= 0x0200; 
    Ch_Faiult2 ++;
  }
  else Ch_Faiult2 = 0;
  
  //22_01_21-сброс через 2 сек
  if((F0_QS1 != 2)) {
    if(T3_QS1>1)T3_QS1--;
    if(T3_QS1<(TT3_QS1-200)){ FAULT &= 0xfffffdff; T3_QS1=0;}//Fault_10=1; Faultec = 1;11_12_19 самосброс отменяется 13_12_19  // разкоментил 25_01_20
  }
  
}



/*-------------------------Управление контактором заряда конденсаторов KM1------------------------*/
#define TT1_KM1 20   //Заданный интервал в циклах для контроля времени включения (отключения)

uint8_t F0_KM1 = 0;     //Состояние KM1: 0 - отключен 1 - включен
uint8_t F1_KM1 = 0;     //Флаг управления циклом включения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F2_KM1 = 0;     //Флаг управления циклом отключения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F8_KM1 = 0;     //Внешнее управление (команда): 0 - нет команды 1 - включить 2 - отключить
uint8_t F9_KM1 = 0;     //Копия F8_KM1 (для цепочных команд)

uint16_t T1_KM1 = 0;    //Счетчик циклов при включении
uint16_t T2_KM1 = 0;    //Счетчик циклов при отключении
uint16_t Ch_KM1=0;      //Счетчик авто-включения КМ1
uint8_t Fault_29 = 0;   //Сгорел СветВВИ
uint8_t f_TP = 1;

extern uint8_t Otkaz_Vkl;
extern int16_t V1_N;

//Состояние KM1
#define inp1_5 ((Input_X1 & 0x0010)>>4)

extern uint8_t Out_X1_4;     //Управление KM1 - Включить
extern uint8_t F2_KQF1;
extern uint8_t F1_KQF1;
extern uint8_t Vkl_VS1_2;
extern uint8_t C12_4;
extern uint8_t F8_KM2;
extern uint8_t Ch_F_02, Ch_F_03 ; //06_05_20
extern uint8_t f_Test;
#define inp4_1 (Input_X2 & 0x0001) //БВ вкл-ть прямо
#define inp4_2 ((Input_X2 & 0x0002)>>1) //БВ вкл-ть через проверку
#define inp4_5 ((Input_X2 & 0x0010)>>4) //разреш. на вкл.
uint8_t TKM1 = 0;
extern uint8_t F8_Proverka;
extern uint8_t F0_QF1;
extern uint8_t f_Avar;  //след

void KM1_Control(void) //10 ms
{
  //if ((inp4_1 || inp4_2) == 1) Trig_Prot &= 0x7fff ; 24_01_20
  // авто-включение КМ1 через 0,64сек ( "|| Fault_08 " --  27_03)
  if ((Out_X1_4==0)&&(((Fault_29 || (Fault_08 || Fault_15) || Fault_12 || Fault_13  // 23_12_21 - "|| Fault_13"
                        || F2_KQF1 || F1_KQF1 ||(f_Zapr_Zar)) ==0)  //inp1_5 поменял на Out_X1_4 было(FAULT &(0x01<<21)) 18.03.24 сделал (FAULT)
      &&( inp4_5==1 )||(f_Test && (f_Zapr_Zar != 0))) && ((FAULT) == 0) || (F0_QF1 != 0))// авто-включение КМ1 через 0,8сек ( "|| Fault_08 " --  27_03) //(Trig_Prot & 0x8000 == 0)
  {
    if(Ch_KM1>80) F8_KM1 =1; // 25_03_19 inp4_5=1; было 64
    else Ch_KM1++;  
  }
  if(inp1_5 == 1) Ch_KM1 = 0; //08_04_24
  
  //отключение при (Trig_Prot & 0x8000 != 0)//отключение при 29-й 
  if (((f_Zapr_Zar) != 0)&&(Vkl_VS1_2 == 0)&&(V1_N > 100)) //20_01_20 //было if (((Trig_Prot & 0x8000) != 0)&&(Vkl_VS1_2 == 0))
  { 
    //F8_KM1 =2; // 09_03_20
    if((f_TP == 0)&&((inp1_1 && inp1_3)==1)&&((inp1_2 || inp1_4)==0)) {
      #ifdef AFB_40
         C12_4 = 1; F8_KM1 = 2; F8_KM2 = 2; //24_12_21
      #else
         Vkl_VS1_2 = 1; 
      #endif
    }
    //if(((inp1_1 && inp1_3)==1)&&((inp1_2 || inp1_4)==0)) Vkl_VS1_2 = 0; //temp 22_03_24
    f_TP = 1; //27_01_20
  } 
  //f_TP = ((Trig_Prot & 0x8000)!=0);
  if((f_Zapr_Zar)==0)f_TP =0;
  
  F9_KM1 = F8_KM1;
  if ((F9_KM1==1)&&(F1_KM1==0)&&(F2_KM1==0)&&(((f_Zapr_Zar) == 0)||(Vkl_VS1_2!=0))&&(((Ch_F_02 > 3)||(Ch_F_03 > 3))==0)&&(Fault_29 == 0)) //31_03_20
  {
    F1_KM1=1;
    Out_X1_4=1;  
    T1_KM1=TT1_KM1;
    T2_KM1=0;
    TKM1= TT1_KM1 - 5; //19_07_22
  }
	
  if (Out_X1_4==1) // было (F1_KM1==1) 13_05_21
  {
    if (inp1_5==1)
    {
      //printf("Контактор КМ1 включен \n");
      T1_KM1=TT1_KM1; // было T1_KM1=0; 13_05_21
      F1_KM1=0;
    }
    else
    {
      if (T1_KM1!=0)
      {
        T1_KM1--;
      }
      else
      {
        //Out_X1_4=0;  //03_04_19
        F1_KM1=0;
         Fault_08=1; Faultec = 1;   //Otkaz_Vkl = 1;05_01_21 по письму 29_12_20 //08_04_24 if(Fault_15==0) {  }
        //printf("Контактор КМ1 неисправен \n");
      }
    }
  }
  
  // по письму СШ 18_07_22
  if( (F1_KM1 || F2_KM1 ) && (Fault_08 == 0)) {  // 08_04_24
    if( TKM1 != 0 ) TKM1 --;
    else { Fault_15 =1; Faultec =1; F1_KM1 =0; F2_KM1 =0; F8_Proverka =0;}
  }
  
  if ((F9_KM1==2)&&((F1_KM1==0)&&(F2_KM1==0)&&(Out_X1_4 == 1))) //||(FAULT & (0x01 << 28)) 17_02_21 //08_04_24
  {
    F2_KM1=1;
    Out_X1_4=0;
    T2_KM1 = TT1_KM1;
    T1_KM1=0;
    TKM1= TT1_KM1 - 5; //19_07_22
  }
	
  if (Out_X1_4==0) // было (F2_KM1!=0) 13_05_21
  {
    if (inp1_5==0)
    {
      //printf("Контактор КМ1 отключен \n");
      T2_KM1 = TT1_KM1; // было T1_KM1=0; 13_05_21
      F2_KM1 = 0;
    }
    else
    {
      if (T2_KM1!=0)
      {
        T2_KM1--;
      }
      else
      {
        F2_KM1=0;
        if(Fault_15==0) {Fault_08 =1; Faultec = 1; }  //Otkaz_Vkl = 1; 05_01_21 по письму 29_12_20
        //printf("Контактор КМ1 неисправен \n");
      }
    }
  }

  //if (inp1_5==0) F0_KM1=0; //данный флаг не применился 13_05_21 оптимизирую
  //else F0_KM1=1;
  
  F8_KM1=0;
  F9_KM1=0;
  
}


/*-------------------------Управление контактором заряда конденсаторов KM2------------------------*/
#define TT1_KM2 20   //Заданный интервал в циклах для контроля времени включения (отключения)

uint8_t F0_KM2 = 0;     //Состояние KM1: 0 - отключен 1 - включен
uint8_t F1_KM2 = 0;     //Флаг управления циклом включения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F2_KM2 = 0;     //Флаг управления циклом отключения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F8_KM2 = 0;     //Внешнее управление (команда): 0 - нет команды 1 - включить 2 - отключить
uint8_t F9_KM2 = 0;     //Копия F8_KM1 (для цепочных команд)

uint16_t T1_KM2 = 0;    //Счетчик циклов при включении
uint16_t T2_KM2 = 0;    //Счетчик циклов при отключении
uint8_t Ch_KM2=0;      //Счетчик авто-включения КМ2
uint8_t Ch_KM2_o=0;
extern uint32_t FAULT;
extern uint8_t F8_Proverka;
uint8_t TKM2 =0;


//Состояние KM2
#define inp1_6 ((Input_X1 & 0x0020)>>5) //Х1В:6 (Контактор ИКЗ вкл.)

extern uint8_t Out_X1_5;     //Управление KM2 - Включить

void KM2_Control(void) //10 ms
{
  
 #ifdef AFB_40 
  
  if ((inp1_4==0)&& inp1_3 &&(Vkl_VS1_2==0)&& inp1_6 ) //авто-выключение 16_11 
  {
    if(Ch_KM2_o>54)F8_KM2=2; //0.54 сек
    else Ch_KM2_o++;
  }
  else Ch_KM2_o =0;
  
 #else
  
  if ((inp1_3==0)&&(inp1_4)&&(inp1_6==0)&&((Fault_29 || F2_KQF1 || F1_KQF1 || (Fault_08 || Fault_15))==0))// авто-включение КМ2 через 0,64сек 12_05_21 //12_08_20 
  {
      if(Ch_KM2>64)F8_KM2=1;
      else Ch_KM2++;
  }
  else Ch_KM2=0;  //05_04_19   //для 4-ки не нужно 
   
  if (((inp1_4==0)||inp1_3) && (inp1_6)&&(((F1_KQF1|| Vkl_VS1_2)==0) || Ch_KM2_o)) //авто-выключение 16_11 //06_05 клацало KM2 // добавил F1_KQF1 
  {
    if(Ch_KM2_o>77)
      F8_KM2=2; //0.77 сек
    else Ch_KM2_o++;
  }
  else Ch_KM2_o =0;
  
 #endif

  F9_KM2 = F8_KM2;
  
  if ((F9_KM2==1)&&(F1_KM2==0)&&((F2_KM2 || (Fault_08 || Fault_15)) ==0)) 
  {
    F1_KM2=1;
    Out_X1_5=1;
    T1_KM2=TT1_KM2;
    T2_KM2=0;
    TKM2 = TT1_KM2 - 5; //19_07_22
  }
	
  if ((Out_X1_5==1)&& ((Fault_08 || Fault_15)==0)) // было (F1_KM2==1) 13_05_21
  {
    if (inp1_6==1) //Х1В:6 (Контактор ИКЗ вкл.)
    {
      //printf("Контактор КМ2 включен \n");
      T1_KM2=TT1_KM2; // было T1_KM1=0; 13_05_21
      F1_KM2=0;
    }
    else
    {
      if (T1_KM2!=0)
      {
        T1_KM2--;
      }
      else
      {
        Out_X1_5=0;
        F1_KM2=0;
        Fault_08=1; Faultec = 1;  //Otkaz_Vkl = 1; 05_01_21 по письму 29_12_20
        F8_Proverka =0;//printf("Контактор КМ2 неисправен \n");
      }
    }
  }
  
    // по письму СШ 18_07_22
  if(( F1_KM2 || F2_KM2 ) && (Fault_08 == 0)) { //09_04_24
    if( TKM2 !=0 ) TKM2 --;
    else { Fault_15 =1; Faultec =1; F1_KM2 =0; F2_KM2 =0; Out_X1_5=0; F8_Proverka = 0;}    //
  }
  
  if ((F9_KM2==2)&&(F1_KM2==0)&&(F2_KM2==0)&&(Out_X1_5 == 1)) //09_04_24
  {
    F2_KM2=1;
    Out_X1_5=0;
    T2_KM2 = TT1_KM2;
    T1_KM2=0;
    TKM2 = TT1_KM2 - 5; //19_07_22
  }
	
  if (Out_X1_5==0)  // было (F2_KM2!=0) 13_05_21
  {
    if (inp1_6==0) //
    {
      //printf("Контактор КМ2 отключен \n");
      T2_KM2=TT1_KM2; // было T1_KM1=0; 13_05_21
      F2_KM2=0;
    }
    else
    {
      if (T2_KM2!=0)
      {
        T2_KM2--;
      }
      else
      {
        F2_KM2=0;
        Fault_08 =1; Faultec = 1; //printf("Контактор КМ2 неисправен \n"); // Otkaz_Vkl = 1; 05_01_21 по письму 29_12_20 //08_04_24  if(Fault_15==0) {Fault_08 =1; Faultec = 1;}
      }
    }
  }

  if (inp1_6==0)
  {
    F0_KM2=0;
  }
  else
  {
    F0_KM2=1;
  }
  //F0_KM2=(inp1_6==0);
  
  F8_KM2=0;
  F9_KM2=0;
}


/*------------------------- вентилятор ------------------------*/

uint8_t F0_Fan = 0;     //Состояние вентилятора: 0 - отключен 1 - включен  F0_Fan = Out_X1_6.
uint8_t F1_Fan = 0;     //Флаг управления циклом включения: 0 - цикл не запущен 1 - цикл выполняется.. 2 - цикл завершен
uint8_t F2_Fan = 0;     //Флаг управления циклом отключения: 0 - цикл не запущен 1 - цикл выполняется.. 2 - цикл завершен

uint16_t T_Fan=0; //счётчик задержки вкл-я/выкл-я вентилятора
//uint16_t T2_Fan=0; //счётчик задержки  вентилятора
uint8_t Ch_Shm=0;
uint8_t Ch_Shm2=0;
#define inp1_7 ((Input_X1 & 0x0040)>>6) // датчик температуры 1
#define inp1_8 ((Input_X1 & 0x0080)>>7) // датчик температуры 2
extern uint8_t F_I2C_Wr; 
//EEProm_Wr =1; Ch_EEProm =50;} //19_03_21 Ch_Zad_Wr_FBit
uint8_t EEProm_Wr =0, Ch_EEProm =0, Ch_Zad_Wr_FBit =0;
uint8_t f_ZaprZar1=0, f_ZaprZar2=0;
//
//uint16_t f_Zapr_Zar = 0; //в начале файла
uint16_t Ch_Zapr_Zar = 350;

void Fan_Control(void) //10 ms   //и сравнение токов по Шмаровозу  //10 ms  27_03  
{
  #ifdef AFB_40
    uint16_t Ivent1 = 4050;
    uint16_t Ivent2 = 3900;
  #else
    uint16_t Ivent1 = 2550;
    uint16_t Ivent2 = 2400;
  #endif
  //вкл-е вентилятора при превышении тока 2500А 
  if((( (int16_t)I1_N_64ms > Ivent1 )||((int16_t)I1_N_64ms < (-Ivent1))|| (inp1_7 == 0)) && (Out_X1_6==0)) // inp1_7 == 1/   13_12_19
  {
    F2_Fan = 0;
    if ( T_Fan ==0 ) 
    {
      if (F1_Fan==0)T_Fan = 100; //при первом проходе
      else  Out_X1_6=1;
    }
    else T_Fan--;
    F1_Fan =  1 ;
  }
  //else
  
  //выкл-е при снижении тока меньше 2450А // 3750 для 4-ки по письму 6_12_21
// отключается на 100А раньше т.е. при 2500
  if((( (int16_t)I1_N_64ms < Ivent2 )&&((int16_t)I1_N_64ms > (-Ivent2))) && (Out_X1_6==1) && (inp1_7 == 1))// inp1_7 == 0/  13_12_19
  {
    F1_Fan = 0;
    if ( T_Fan ==0 ) 
    {
      if (F2_Fan==0)T_Fan = 6000; //при первом проходе
      else  Out_X1_6=0;
    }
    else T_Fan--;
    F2_Fan = 1 ;
  }
  
  
  //сравнение токов по Шмаровозу  //10 ms  27_03  
  /*if ((((int16_t)I_SMTN - (int16_t)I1_N_64ms)>(int16_t)250)
      ||(((int16_t)I1_N_64ms - (int16_t)I_SMTN)>(int16_t)250))
  { 
    if (((int16_t)I1_N_64ms > (int16_t)300)||((int16_t)I1_N_64ms < (int16_t)(-300))){
    Ch_Shm++;
    //if(Ch_Shm > 10)FAULT |= 0x10000;//Fault_17=1; закоментировал для тестов
    } 
  }
  else Ch_Shm=0;*/
  
  if ((((int16_t)I1_N_64ms > (int16_t)100)||  // ток при отключенном БВ то -- Fault_17=1
       ((int16_t)I1_N_64ms < (int16_t)(-100))) && ((inp1_3)||(inp1_4==0))){
   	Ch_Shm2 ++;
  	if(Ch_Shm2 > 20) { FAULT |= 0x10000; } //if(F1_QS1)F8_QS1=2;
      
  } //Fault_17=1; //19_03_21 добавил if(F1_QS1)QS1=2;
  else Ch_Shm2 = 0;
  
  ///////// сохранение в EEPROM по флагу //////// 01_09_20
  if ( Status_Bit & 0x0008 ) { 
    if(EEProm_Wr == 0) {F_I2C_Wr = 1; EEProm_Wr = 1; Ch_EEProm = 50;} //19_03_21 Ch_Zad_Wr_FBit
    if(Ch_EEProm <= 0) {Status_Bit &= (~0x0008); F_I2C_Wr = 2;}//сброс бита через 0,5сек
    if(Ch_EEProm == 25) F_I2C_Wr = 2;
    Ch_EEProm --;
  }
  /// задержка сброса бита записи 19_03_22
  if(EEProm_Wr) Ch_Zad_Wr_FBit--;
  if(Ch_Zad_Wr_FBit <=0) EEProm_Wr =0;
  
  /// отдельный бит запрет заряда  
  uint8_t ZprZr = ((Trig_Prot&0x8000)!=0); //для старых проэктов
  if(ZprZr != f_ZaprZar1){ 
    Reg_ZprZr  = ZprZr; 
    f_ZaprZar1 = ZprZr; 
    f_ZaprZar2 = ZprZr; }
  
  if(Reg_ZprZr != f_ZaprZar2){ 
    Trig_Prot &= 0x7fff; 
    Trig_Prot |= (Reg_ZprZr&0x01) << 15; 
    f_ZaprZar1 = (Reg_ZprZr&0x01); 
    f_ZaprZar2 = (Reg_ZprZr&0x01); } //29_07_21 Trig_Prot |=
  
  //флаг запрет заряда 18.03.24
  if(((Trig_Prot & 0x8000) == 0) && (Ch_Zapr_Zar > 349)) {
     f_Zapr_Zar = 0; } //Ch_Zapr_Zar = 0;
  if(((Trig_Prot & 0x8000) != 0) && (f_Zapr_Zar == 0)) Ch_Zapr_Zar = 0;
  if(((Trig_Prot & 0x8000) != 0) || (Ch_Zapr_Zar < 350)) 
    f_Zapr_Zar = 0x8000; //f_Zapr_Zar = 1;
	//else f_Zapr_Zar = 0;
  if((Ch_Zapr_Zar < 350) &&(f_Zapr_Zar != 0)) 
    Ch_Zapr_Zar ++;
  
}




/*--------------------проверка готовности и исправности AFB-------------------------*/
uint8_t F_Cap_11 = 0;           //Состояние заряда конденсатора С1:1   -> 0-не заряжен, 1-заряжен 840-1100
uint8_t F_Cap_12 = 0;           //Состояние заряда конденсатора С1:2   -> 0-не заряжен, 1-заряжен 965-990
//uint8_t F_VS1_Good = 0;         //Результат испытания (при включ-е) работы тиристора VS1
uint8_t F_VS2_Good = 0;         //Результат испытания (при включ-е) работы тиристора VS2
uint8_t F_VD2_Good = 0;         //Результат испытания (при включ-е) исправности диода VD2  (Opto_IN1)
uint8_t F_Modbus_Good = 0;      //Результат проверки (при включ-е) связи по Modbus
//uint8_t F_IGCT_Good = 0;        //Результат проверки (при включ-е) IGCT для 4к-версии
uint8_t F_AFB_Good = 0;         //Результат проверки AFB: 1- готов; 0- неготов или ошибка.
uint8_t Ch_F_Cap1=0;
uint8_t Ch_F_Cap2=0;
uint16_t Ch_F_Cap3=0;
uint16_t Ch_Puls_X4_1=0;
uint16_t Ch_F29 = 0;
uint16_t Ch_F29_2 = 16;
uint8_t Ch_Per = 0;
uint8_t F_Cap_Contr = 0;
extern uint8_t F0_QF1;
extern uint8_t F2_QF1;
extern int16_t V1_N,V2_N,V3_N;
extern uint8_t F_Vkl_Prov;
uint8_t F1_KQF1=0;
uint8_t F2_KQF1=0; 
uint32_t F_Otkl=0; //ошибка по которой произошло отключение БВ, с разъединителем
uint8_t Ch_F_02=0; //06_05_20
uint8_t Ch_F_03=0; //06_05_20
uint16_t T_02_03=0;//06_05_20
uint8_t Ch_IGCT_Got =0;
uint8_t f_DDC=0;
uint16_t Ch_CapC =0;
extern uint8_t f_Test;
extern int16_t I2_N_64ms;
uint8_t Ch_ZR = 0;  //Счетчик защиты резисторов
uint16_t Ch_ZR1 = 0; //Счетчик защиты резисторов 4k
uint8_t Ch_I2_ = 0;
uint8_t Out_X1_4_Old = 0;

#define inp1_9 ((Input_X1 & 0x0100)>>8) // датчик давлен

#ifdef AFB_40 
  extern uint8_t IGCT_Test;
  extern uint16_t Ch_IGCT;
  uint16_t Ch_IGCT2 = 0;
  extern uint8_t Opto_IN2;
  extern uint8_t Opto_IN3;
  extern uint8_t FAZA_Otkl;
  uint16_t Ch_Vkl_Prov = 500;
#endif

void Cap_Control (void) //10 ms  и проверка при включение IGCT и датчик давления конд-в
{ // 
  if (((V1_N>(int16_t)860) && (V2_N>(int16_t)980)) && ((F1_KQF1||F2_KQF1) == 0) // 9_04_21 TEMP
	  && ((V1_N<(int16_t)1100) && (V2_N<(int16_t)1100))) 
    F_Cap_Contr = 1; //14_11 исключение Fault_02, 03 при разряде конденсаторов во время выключения QF1, (проверка перед вкл QF1)
  
  //04_11_20 //разкометил 29_04_21
  if (((F1_KQF1||F2_KQF1||F_Cap_Contr) == 0)&&(inp1_5==1)){
    Ch_CapC ++;
    if(Ch_CapC > 500)
      F_Cap_Contr = 1;
  }
  else Ch_CapC =0; // 
  
  if((Out_X1_4 == 1)&&(Out_X1_4_Old == 0))  //22_03_24
    F_Cap_Contr = 0;
  Out_X1_4_Old = Out_X1_4;
  
#ifdef AFB_40 
  //-проверка при включение // 31_05_21 - объед 01 и 05   << 4
  // 29_07_21 задержка проверки 5сек после запуска системы
  if(Ch_Vkl_Prov) Ch_Vkl_Prov --;
  if(F_Cap_Contr && (IGCT_Test ==2) && (inp1_1==1) && (inp1_2==0) && (Ch_Vkl_Prov == 0)) { //19_05_20 && (Opto_IN2 ==0  
     if((Opto_IN2 == 0) && ((FAULT & 0x01)==0) && (Ch_IGCT2 > 100)) {
       IGCT_Test = 1; Ch_IGCT = 200; Ch_IGCT2 = 0; Ch_CapC = 0;} //14_07_2020 //9_04_21 - Ch_IGCT = 2000; поменял на 200
     else Ch_IGCT2 ++;
     if (Ch_IGCT2 > 250) 
       FAULT |= 0x01 ; //19_05_20
   }//при первом заряде конд-в для проверки IGCT 23_12_19
  
  //контроль готовности IGCT 15_07_20
  //if (Opto_IN2 && (inp1_3==0) && (inp1_4==1))
  //{
  //   if (Ch_IGCT_Got) {FAULT |= 0x01; Out_X4_6=1;}
  //   Ch_IGCT_Got ++;}
#endif
  
int16_t  V1_N_maks = 2700;
  #ifdef AFB_40
    V1_N_maks = 1700;
  #endif
  
  //запрет отключения по заряду конденсаторов
  F_Cap_11 = (V1_N>(int16_t)840)&&(V1_N<(int16_t)1200);
  F_Cap_12 = (V2_N>(int16_t)960)&&(V2_N<(int16_t)1200);  //было(V2_N>965)
  
  //ошибки 02,03 //при принуд-м разряде не проверять ((Trig_Prot & 0x8000)==0) 10_08_20
  if (((V1_N<(int16_t)860)||(V1_N>(int16_t)V1_N_maks)) && F_Cap_Contr && ((f_Zapr_Zar)==0)
      &&((FAULT & 0x08)==0)&&(Out_X1_4==1)) { //17_12 //27_04_21 //07_06_21 //20_12_21 -СШ сломал алгоритм добав. "&&(Out_X1_4==1)"
    if (Ch_F_Cap1 > 3) {
      if((FAULT & 0x0002)==0) {
        Ch_F_02 ++; //06_05_20  //убрал 04_11_20 - F8_KM1=2;
        //if((V1_N>(int16_t)1100)&&(F0_QF1 == 0))F8_KM1=2;  04_11_20
      }
      FAULT |= 0x0002; //Fault_02 =1; Faultec = 1;
      T_02_03 = 3000; //06_05_20
      F_Cap_Contr =0; //14_04_21
      if((FAULT & 0x01)&&((inp1_3 == 1)&&(inp1_4 == 0)&&(inp1_1 == 0)&&(inp1_2 == 1))) Out_X4_6 = 1; //10_06_21
    }
    else Ch_F_Cap1++;
  }
  else {
    Ch_F_Cap1=0; 
    if((F_Cap_Contr !=0) &&(Ch_F_02 < 4)){ //06_05_20
      //FAULT &= 0xfffffffd;  //  тарасу л. //14_04_21
      F_Otkl &= 0xfffffffd; //?
  }} 
  
  if (((V2_N<(int16_t)980)||(V2_N>(int16_t)1200)) && F_Cap_Contr && ((f_Zapr_Zar)==0)
      &&((FAULT & 0x08)==0)&&(Out_X1_4==1)) { //17_12 //27_04_21 //07_06_21 //18_08_21 порог до 1200 //20_12_21 -СШ сломал алгоритм добав. "&&(Out_X1_4==1)"
    if (Ch_F_Cap2 > 3) { 
      if((FAULT & 0x0004)==0) {
        Ch_F_03 ++; //06_05_20  //убрал 04_11_20 - F8_KM1=2;
        //if((V2_N>(int16_t)1100)&&(F0_QF1 == 0))F8_KM1=2;  04_11_20
      }
      FAULT |= 0x0004; //Fault_03=1; Faultec = 1;
      T_02_03 = 3000; //06_05_20
      F_Cap_Contr=0; //14_04_21
    }
    else Ch_F_Cap2++;
  }
  else {
    Ch_F_Cap2=0; 
    if((F_Cap_Contr !=0) &&(Ch_F_03 < 4)){ //06_05_20
      //FAULT &= 0xfffffffb; //14_04_21
      F_Otkl &= 0xfffffffb;
  }}
  //Таймер №сек
  if(T_02_03) T_02_03 --;
  else {Ch_F_02=0; Ch_F_03=0;}
  
  if ((((V1_N>(int16_t)840) && (V2_N>(int16_t)960)) == 0) && (Out_X1_4==1)) //(F_Cap_11 && F_Cap_12)//22_03_24 было && (inp1_5) стало &&(Out_X1_4==1)
  { Ch_F_Cap3++; if (Ch_F_Cap3 > 900){ if (F_Cap_11==0) FAULT |= 0x0002; if (F_Cap_12==0) FAULT |= 0x0004; }  }
  else Ch_F_Cap3=0;
 
   
  //Проверка исправности ВВСвет-БП
  if(inp1_5 && (((V1_N>(int16_t)840) || (V2_N>(int16_t)970) || f_Test)==0) && ((Fault_15 || Fault_08)==0)) //#define inp1_5 ((Input_X1 & 0x0010)>>4)
  {
    Ch_F29++; //
    if (Ch_F29>1500) {Fault_29=1; Faultec = 1; F8_KM1 = 2;}//10сек задержка TEMP 21_11
  }
  else Ch_F29=0;
  
  //идея СергШ 12_11 - отключение БВ при разряде конденсаторов
  if ((V2_N<980)&&(F0_QF1==1) && F_Cap_Contr ) //970 на 980 27_04_21
  {
    Ch_F29_2--;
    if (Ch_F29_2 < 1) {Fault_29 = 1; Faultec = 1; F8_KM1 = 2;}//160мсек задержка
  }
  else Ch_F29_2 = 16;
  
  //защита от перенапряжения на конденсаторе
  /*if ((V1_N>(int16_t)1100) && inp1_1) 
  { Ch_Per++; //if(Ch_Per > 32)F8_KM1=2; //F_Perenapr =1;
   } 
  else Ch_Per=0;
  if ((V1_N<(int16_t)(-1200)) && inp1_1) F_Perenapr =2;*/
  //датчик давления
  if ((inp1_9 == 0)&&((Trig_Prot & 0x0001) == 1)) {FAULT |= (0x01 << 21); F8_KM1 = 2;}
  
uint8_t I2_N_Lim = 50;
  #ifdef AFB_40
  I2_N_Lim = 80;
   //защита резистора по СШ 18_05_21
  if((Out_X4_6==0)&&(((inp1_3 ==1)&&(inp1_4==0))&&((inp1_2==1)&&(inp1_1==0)))
     &&(Ch_IGCT == 0) ) //(&&(V1_N<(int16_t)860) || ((FAULT&0x08)!=0))
  {
    if( ((I2_N_64ms > I2_N_Lim) || (I2_N_64ms < (-I2_N_Lim))) && ((FAZA_Otkl > 6) || (FAZA_Otkl == 0)) //((FAZA_Otkl > 6) || (FAZA_Otkl == 0)) & 1-6
       && ((Out_X1_5 && Out_X1_4)==0) &&((F1_KQF1||F2_KQF1)==0) ) {  //26_07_21 срабатывала 20-я ошибка при вкл.
      if( Ch_I2_ > 10 ) {   // 20_08_21 с 4-х поменял на 10
        Fault_20 = 1; Faultec = 1; Out_X4_6 = 1;}
      Ch_I2_ ++;
    }
    else Ch_I2_ = 0;
    
    if((Opto_IN2) && F_Cap_Contr) { //F_Cap_Contr(Ch_IGCT==0) //10_06_21
      Ch_ZR1++;
      if(Ch_ZR1 > 8)
      { Faultec =1;
        //Fault_26 |= Opto_IN3; // 26-или другая 10_06_21
        FAULT |= Opto_IN2; //18_05_21
        //if((I2_N_64ms > 80) || (I2_N_64ms < (-80))) Fault_20 = 1;
        if(((FAULT&0x01)&&(Fault_20))) Out_X4_6 = 1; //||Fault_26 ||(Fault_20&&Fault_26) - убрал по желанию СШ, ему не нравится что долго работает
      } 
    }
  }
  else Ch_ZR1 =0;
  #endif
   //#else
    //защита резистора по СШ -проверка тока ТА2 и контакторов 13_05_21 &&(Ch_IGCT == 0)
  if((Out_X4_6==0)&&(Out_X1_5==0)&&(((((I2_N_64ms > I2_N_Lim)||(I2_N_64ms < (-I2_N_Lim)))&&((Out_X1_4&&Out_X1_5)==0)) //&&((F2_KQF1||F1_KQF1) == 0)
                     ||(inp1_5&&inp1_6&&Fault_08))&&(((inp1_3 ==1)||(inp1_4==0))&&((inp1_2==1)&&(inp1_1==0))))&&((F1_KQF1||F2_KQF1)==0))
  {
    Ch_ZR++;
    if(Ch_ZR > 3){
      Out_X4_6=1; 
      Fault_20 = ((I2_N_64ms > 50)||(I2_N_64ms < (-50))); 
      //Fault_15 = inp1_5&&inp1_6 ; //08_04_24
      Faultec = 1; }
    if(Ch_ZR > 1){F8_KM1 = 2;} //F8_KM2=2;
  }
  else Ch_ZR = 0;
  
  //09_06_21
  //if((F_AFB_Good) && (IGCT_Test==2)) f_SSh =1; // в xxx_it.c
}


/*--------------------  AFB - QF1 on/off -------------------------*/
extern uint16_t T_Vkl;
extern uint8_t FAZA_Vkl;
extern uint16_t T_Otkl;   
extern uint8_t FAZA_Otkl;
extern uint8_t F8_QF1;
extern uint8_t Otkaz_Vkl; //Fault_14
extern uint8_t Otkaz_IKZ;
//extern uint16_t P_Io_Set;
//extern uint16_t N_Io_Set;
extern int16_t I1_N;
extern int16_t I2_N;
uint8_t F_Over_Current=0;
extern uint8_t Fault_04;
extern uint8_t Fault_05;
extern uint8_t Fault_06;
extern uint8_t Fault_13;
extern uint8_t Fault_16;
uint8_t Fault_14;
extern uint8_t Fault_17;
extern uint8_t Fault_18;
extern uint8_t Fault_27;
extern uint8_t Fault_28;
										   

#define inp4_1 (Input_X2 & 0x0001) //БВ вкл-ть прямо
#define inp4_2 ((Input_X2 & 0x0002)>>1) //БВ вкл-ть через проверку - вх. блокир. вкл-я
#define inp4_3 ((Input_X2 & 0x0004)>>2) //БВ отключить с разъед-м
#define inp4_4 ((Input_X2 & 0x0008)>>3) //БВ отключить с без
//#define inp4_5 ((Input_X4 & 0x0010)>>4) //разреш. на вкл.
#define inp4_6 ((Input_X2 & 0x0020)>>5) //сброс неисправностей
#define inp4_7 ((Input_X2 & 0x0040)>>6) //питание 24В с ячейки  - авар. откл.
#define inp4_8 ((Input_X2 & 0x0080)>>7) //авар. откл.	- розряд конд-в


extern uint8_t F6_Faza;
extern uint8_t F5_Faza;
extern uint8_t f_Zapusk; //флаг обнуления флагов при запуске
extern uint8_t F_I2C_Wr;
//extern uint8_t STATUS;
//extern uint16_t Status_Flag;

uint8_t Funk_Otkl=0;
uint8_t F_Zapis; //флаг сохр. уставок перед выкл-м
uint32_t FAULT;  //все ошибки
uint32_t Prichina_Otkl=0; //ошибка по которой произошло отключение БВ, без разъединителя
uint8_t F_Vkl_Prov = 1;   //проверка при включение
//uint16_t Ch_Vkl_P=0;
uint8_t Ch_F5=0;

uint8_t F_Ch_Otkl=0;
uint8_t f_Sbros=0;
extern uint8_t f_SPI1_;
extern uint8_t f_SPI2_;
//extern int16_t I1_Otkl; 
uint32_t FAULT_V = 0; //ошибки перед отключением БВ, запись в ППЗУ и Модбас
uint8_t Ch_Out_X4_7 = 0; //выход блокировки,для паралельной работы БВ
uint8_t f_Vkl = 0;
uint8_t inp4_4_Pred=0;
extern uint8_t f_Test;
uint8_t Par_Otkl = 0; //Флаг отключения паралельного выключателя
uint8_t Ch_Par_O = 0; //Счетчик отключения паралельного выключателя
uint8_t f_Blok = 0; //Флаг блокировки отключения паралельного выключателя
uint16_t Ch_Urov_Vk =0; //Задержка на выставление УРОВ по письму 08_07_20
uint8_t BV_Otkl=0; //флаг - БВ отключен, при комманде откл. с БВ
uint8_t Zbross =0; //30_10_20 для задержки сигналла сбросса
uint8_t Ch_Zbross =0;
uint8_t Ch_Otkaz_Otkl_1; //06_01_21
uint8_t Ch_Otkaz_Otkl_2; //11_01_21
uint8_t f_SSh; //08_06_21
uint8_t f_Qwer = 0; //06_07_22
//новые перем. для алг-ма по письму 22_04_22
/*uint8_t Out_V1 =0; //77
uint8_t Out_V2 =0; //78
int16_t Inp_V1 =0; //81
int16_t Inp_V2 =0; //82
int16_t U1_V =0;  //83
int16_t U2_V =0;  //84*/
extern uint8_t TxDO[6];

extern uint8_t Ch_Start_Sbr;
extern uint8_t Sled_Wr;
////////////////////////////////////////////////////////////////////////////////

void Algoritm (void) //50 mks 
{
  ////TEMP test tiristor
  //GPIOE->BSRR = (GPIO_BSRR_BS13 | GPIO_BSRR_BS14 | GPIO_BSRR_BS15);
  
  if (f_Test == 0){
  ///команды вкл./откл.************************************|| Otkaz_IKZ
  //включение  (((Input_X4&0x03)!=0)&&((Input_X1&0x0C)==4)&&(F_AFB_Good==1)&&((Input_X4&0x8C)==0)&&(F2_KQF1 || F1_KQF1||Otkaz_Vkl||F_Over_Current==0))
  if (((inp4_1 || inp4_2) == 1)&&(inp1_3==1)&&(inp1_4==0)&&(F_AFB_Good==1)
	  &&(Otkaz_Vkl ==0)&&((inp4_3 || inp4_4 || inp4_8 || f_Vkl) ==0)&&((F2_KQF1 || F1_KQF1) ==0)&&(F0_QS1 != 2)) //||F_Over_Current
  { 
    Status_Flag &=0xfe00; //10_08_20 убрал 17_02_21  а 18-го вернул
    Otkaz_IKZ = 0; F_Over_Current = 0;//25_01_20 - сказал СШ обнулять и убрать запрет вкл по ИКЗ //17_02_21 - Otkaz_IKZ = 0; F_Over_Current = 0; убрал а 18-го вернул
    Prichina_Otkl &= 0xfffffffe; F_Otkl =0;// 12_06 
    //F_Cap_Contr=0;  // блокировать проверку ошибок 02,03 пока идет включение/выключение //14_04_21
    if (inp4_1==1) {F1_KQF1 = 1; } // printf("запуск процесса включения БВ прямо \n");//Message (1);
    else {F1_KQF1 = 2;  } //printf("запуск процесса включения БВ с проверкай \n"); //Message (2);
    T_Vkl=0; FAZA_Vkl=1; F6_Faza = 1; 
    if ((inp1_1==0)&&(inp1_2==1)) { FAZA_Vkl=3; T_Vkl=9700; } // printf("разъед-ль замкнут \n"); //Message (3);
  
    if (FAULT_V !=0) { //по письму от 22_04_22
      FAULT_V = 0;
      Out_V1 = 0;
      Out_V2 = 0;
      Inp_V1 = 0;
      Inp_V2 = 0;
      U1_V  =  0;
      U2_V  =  0; I1_V  =  0;
      I2_V  =  0; }
	Sled_Wr = 1; //запись следа
  }
  f_Vkl = (inp4_1 || inp4_2 ); 
  
  //отключение с QS1  // || (FAULT&0x0036848e) //(FAULT&0x0036868e) 11_11_19 /0x0036828E 18_12_19  // (F1_KQF1 ||)//0x6036869f - 29_04_21
  if ( ((f_SSh || inp4_3 || inp4_8 || ((FAULT&0x7DB7F7DF)&&(F_Cap_11 && F_Cap_12))) != 0) //&((uint32_t)~F_Otkl) // f_SSh - 08_06_21
	  &&(F2_KQF1 ==0)&&((F_Otkl == 0)) ) ////&&(inp1_3 || inp1_1 ==0) а когда БВ выкл-н, разъед вкл - и надо его выкл //11_05_21
  { f_SSh =0; // f_SSh - 08_06_21
    F1_KQF1 = 0; //  19_12 - включение на КЗ
    //Out_X4_7 = (F_Over_Current==0); // по письму 04.09.19 //16_06_20
    Zapis_Sobitij (0x0b);
    Vkl_VS1_2 = 0; //20_03_24
    if (FAULT_V ==0) { //по письму от 22_04_22
      FAULT_V = FAULT;
      Out_V1 = TxDO[0];
      Out_V2 = TxDO[3];
      Inp_V1 = Input_X1;
      Inp_V2 = Input_X2;
      U1_V  =  V1_N;
      U2_V  =  V2_N; 
      I1_V  =  I1_N;
      I2_V  =  I2_N;
    }
    
    if(FAULT!=0) { Zapis_Sobitij (0x07); }
    if ((Ch_Otkaz_Otkl_1 || Ch_Otkaz_Otkl_2)==0){  
      I1_Otkl = I1_N; //02_09 ток перед отключением
      F_Otkl =  FAULT & 0x7DB7F7DF; //Fault_18 || Fault_16 // 11_12_19
      F2_KQF1 = 1;
      T_Otkl =0;   
      BV_Otkl =0; 			//19_05_21
      FAZA_Otkl =0; 		//10_05
      F5_Faza = 1; //printf("запуск процесса откл-я БВ с разъед. \n"); //Message (4);
      if ((inp1_3==1)&&(inp1_4==0)) { //13_01_21 сделал условие по и
          FAZA_Otkl=4; BV_Otkl=1; //Ch_ZR1 =400;
          if((inp1_1==1)&&(inp1_2==0)) F2_KQF1 = 0; //23_09_20
      } //printf("запуск процесса отключения БВ \n"); //Message (5);
      else if (Fault_12 == 0) {			 //11_01_21 -- || Fault_30 || Fault_31
      //F8_QF1=2; 
          if((((FAULT & 0x01))==0)&&(F_Cap_11 && F_Cap_12))F8_QF1 = 2; //&&(F_Cap_11 && F_Cap_12)или вообще в Proverka_Zashit(void)
          else {
          if(((I1_N < (int16_t)300) && (I1_N > (int16_t)(-300)))&&(F_Cap_11 && F_Cap_12))F8_QF1 = 2;
          else {
            Out_X4_6 = ((inp1_3==0)&&(inp1_4==1)); F2_KQF1 = 0; f_Qwer = 1;} //FAULT |= 0x2000; //12_05_21 попросил подпортить ПО С шмаровозз добавил условие ((inp1_3==0)&&(inp1_4==1))
          }
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); //Out_X4_7 = (F_Over_Current==0); 	//или вообще в Proverka_Zashit(void)// по письму 04.09.19 //16_06_20
        F_Cap_Contr=0; }			//26_05_21 //19_03_24
      F_Ch_Otkl =1; 				// 25_02
    //else - сообшение для ПП-что откл-е не возможно?
    }
    else {
        Out_X4_6 = 1; FAULT |= 0x2000; 
        if(Ch_Otkaz_Otkl_1)Fault_31=1; 
        if(Ch_Otkaz_Otkl_2)Fault_30=1; Faultec=1; // 11_01_21 //закоментил 27_04_21 по СШ //разкоментил 28_04_21 СШ одумался
      }
   }
  
    if(inp4_4  || inp4_3 || inp4_8) F1_KQF1 = 0; //прерывание команды включения
  
    //отключение без QS1 || (FAULT&0xff497271) //11_11_19  (0xff7d77ff)
    //uint8_t f_otkl = (Prichina_Otkl==0) && (F_Over_Current || (FAULT&(~0x0036829e))); // 0xff7ff7f9 - маска ошибок которые приводят к отключению
    uint8_t f_otkl = (((uint32_t)(F_Over_Current | (FAULT&(~0x7DB7F7DF))) & (uint32_t)(~Prichina_Otkl))!=0);
    uint8_t Otkl = (((inp4_4 && (inp4_4_Pred==0)) || f_otkl)==1) && ((inp1_3==0)|| F_Over_Current || ((FAULT & 0x0400)!=0) ); //|| inp4_8
    if ( Otkl && ((Fault_12 || F2_KQF1)==0) && (F_Cap_11 && F_Cap_12) && (Prichina_Otkl==0))// || Fault_30 || Fault_31// F1_KQF1 ||  TEMP 19_12 //10_08_20
    { 
      inp4_4_Pred = inp4_4;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); //Out_X4_7 = (F_Over_Current==0); //(F_Over_Current==0)//16_06_20
      I1_Otkl = I1_N; //02_09 ток перед отключением
      Zapis_Sobitij (0x08);
      
      if (FAULT_V ==0) { //по письму от 22_04_22
        FAULT_V = FAULT;
        Out_V1 = TxDO[0];
        Out_V2 = TxDO[3];
        Inp_V1 = Input_X1;
        Inp_V2 = Input_X2;
        U1_V  =  V1_N;
        U2_V  =  V2_N;  
        I1_V  =  I1_N;
        I2_V  =  I2_N; }
    
      //Prichina_Otkl = FAULT | F_Over_Current; //11_10_21
      if(FAULT!=0) { Zapis_Sobitij (0x07);  } //FAULT_V = FAULT;
      if( (Ch_Otkaz_Otkl_1 || Ch_Otkaz_Otkl_2)==0 ) //11_10_21
      {
        Prichina_Otkl = FAULT | F_Over_Current;
        F8_QF1  = 2; //или вообще в Proverka_Zashit(void)
        F2_KQF1 = 2;
        F1_KQF1 = 0; //  19_12 - включение на КЗ
        T_Otkl  = 0;   
        FAZA_Otkl = 0; //11_06
        F5_Faza = 1;
        F_Cap_Contr = 0;
        F_Ch_Otkl = 1; // 25_02
      }
      else {
        Out_X4_6 = 1; FAULT |= 0x2000; if(Ch_Otkaz_Otkl_1)Fault_31=1; if(Ch_Otkaz_Otkl_2)Fault_30=1; Faultec=1; //закоментил 27_04_21 по СШ //разкоментил 28_04_21 СШ одумался
      }//printf("запуск процесса откл-я БВ без разъед.\n"); //Message (6);
    BV_Otkl=0;
    }
    
    if(inp4_4_Pred) inp4_4_Pred = inp4_4; //срабатывание по фронту ( откл. и сброс одной кнопкай)
        
    /// по письму СШ 08_07_20 //задержка 100мсек 
    if ( ((F_Cap_11 && F_Cap_12)==0) && ((inp1_3==0)&&(inp1_4==1)) 
        && ( F_Over_Current || inp4_3 || inp4_4 || inp4_8 || ((FAULT & 0x140000)!=0) ))
    {
      if  (Ch_Urov_Vk > 1500){  //21_07_20 была задержка 100мс(2000), сделал 75мс(1500)
        Out_X4_6=1;  } //FAULT |= 0x800000; F_24
      else Ch_Urov_Vk ++;
    }
    else Ch_Urov_Vk =0;
    
    //// по письму СШ 05_07_22
    if ((I1_N < (int16_t)300) && (I1_N > (int16_t)(-300))  && (Out_X4_6 == 1) && ((FAULT&1) == 1) && f_Qwer) {
      F_Otkl = 0;//откл-е БВ
      f_Qwer = 0;
    }
    if((f_Vkl == 0)||(f_Zapr_Zar )) { if(F1_KQF1 != 0)f_SSh = 1 ; } // Письмо 15_03_24 //прер-е процесса вкл-я если выставлен бит ЗапрЗар? //22_03_24
  } //конец if(F_Test ==0)
  else
  {
    //Тестовый режим для настройки механики
    //включение  && F_AFB_Good&&(Otkaz_Vkl ==0)
    if ((((inp4_1 || inp4_2 ) && inp1_3 )== 1)&&(inp1_4==0)&&((inp4_3 || inp4_4 || inp4_8 || f_Vkl) ==0)) 
    { 
      F8_QF1 = 1;
    }
    f_Vkl = (inp4_1 || inp4_2 );
    

    //отключение без QS1 
    if  ( ((inp4_4 || inp4_3)==1) && (inp1_3==0) && (F2_KQF1 == 0) ) //|| (Prichina_Otkl==0) && (F_Over_Current ) // // F1_KQF1 ||  TEMP 19_12 
    { 
      F8_QF1 = 2; 
      Prichina_Otkl = F_Over_Current; // 06_05_20
    }
    
  }
  
  //сброс сигнала Х4_7   ***** 20_09_19 / mod письмо-16_06_20
  if (Ch_Out_X4_7 > 0){ 
    Ch_Out_X4_7 --;
    if (Ch_Out_X4_7 == 0) // TEMP
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); //Out_X4_7=0; 
     }
  
  //паралельная работа
  //if(SPI2_Priem & 0x4000) //16_06_20
  if(inp4_7)  //28_04_21
  {
    if(Ch_Par_O < 100){
      Ch_Par_O ++;
      Par_Otkl = 1; } //игнорируем сигнал блокировки через 5мсек 
    else Par_Otkl = 0;
  }
  else {Par_Otkl = 0; Ch_Par_O =0; }
  //else Ch_Out_X4_7 = 0;
  //по письму 22_06_20
  if (Par_Otkl) f_Blok = 1;
  if (f_Blok && (Par_Otkl ==0)) { 
    if((((int16_t)dI_dt_1_6ms < 0)&&(I1_N < 0))||(I1_N > 0))
    f_Blok =0;
  }
  
  
  ///команды вкл./откл. end ************************************
  
  
  ///состояние, вых-е сигналы
  //Out_X4_1=F_AFB_Good; //сист. упр. готова - отсутствие ошибок и наличие связи по RS485? - мигание
  Out_X4_2 = F_AFB_Good; //заряд конд-в, в процессе вкл/откл???? 
  Out_X4_3 = F_Over_Current; //свехток и все остальные защиты
  Out_X4_4 = (FAULT !=0); //Otkaz_Vkl; //Fault_14  //03_04_19 //19_03_21
  Out_X4_5 = Otkaz_IKZ; // Переделать!! на "Блокировка QS1"
  
  //Fault_05=inp4_7; // отсутствует напр-е ячейки  
  
 /* if ((inp4_7==0)&&(Fault_05==0)){ 
      if ( Ch_F5 > 32){ Fault_05=1; Faultec=1; }
      Ch_F5++;} //TEMP 
    else Ch_F5=0;*/
  if ((Otkaz_Vkl || Out_X4_6)&&(Fault_14==0)){
    Fault_14=1; Faultec=1;}
  
 ////////////////////////27_05_20

  //сброс неисправностей по вх. сигналу || при включение || по Модбасу
  if(inp4_6) {  //30_10_20
    if(Zbross == 0) Ch_Zbross ++; //30_10_20
    if(Ch_Zbross > 32)Zbross =1;} //30_10_20
  else {Ch_Zbross =0; Zbross =0;} //30_10_20
  
  if(( ((Zbross) && (f_Sbros == 0)) || f_Zapusk) || (Status_Bit&0x0002) ) //&& f_Sbros 
  {
    //f_Sbros = (( inp4_6 || (Status_Bit&0x0002))==0); //Status_Flag =0; 06_10_20 TEMP 
    FAULT_Modbus1 =0; FAULT_Modbus2 =0;   f_SPI1_=0; f_SPI2_=0;
    Otkaz_Vkl=0;  Prichina_Otkl=0; Out_X4_6=0; Faultec=1; Otkaz_IKZ=0; F_Over_Current=0; Status_Flag &=0xfe00;// 17_02_21 добавил Status_Flag &=0xfe00;
    Fault_04=0; Fault_06=0; Fault_07=0; Fault_08=0; F_Otkl=0; Out_X4_4 =0;
    Fault_09=0; Fault_10=0; Fault_12=0; Fault_13=0; Fault_14=0; Fault_15=0; Fault_16=0; //Fault_17=0;
    Fault_18=0; Fault_19=0; Fault_20=0; Fault_21=0; Fault_23=0; Fault_24=0; 
    Fault_25=0; Fault_26=0; Fault_27=0; Fault_29=0; Fault_30=0; Fault_31=0;//TEMP  Fault_28=0;-сброс при снятии пит.
    Ch_F_02=0; Ch_F_03=0; 
    Ch_F29_2 = 5000; //20_07_20
    #ifdef AFB_40
    FAULT &= 0x08000000;
    #else
    Fault_28=0; FAULT =0;
    #endif
  }
  
  f_Sbros = Zbross ; //30_10_20
  
  //объединение ошибок ?
  FAULT |=  Fault_04<<3 | Fault_06<<5;
  if (Faultec) { Out_X4_6 |= Fault_23; //12_03_19
  FAULT |=    Fault_07<<6 | Fault_08<<7 | Fault_09<<8 | Fault_10<<9 | Fault_12<<11 | Fault_13<<12 |
      Fault_14<<13 | Fault_15<<14 | Fault_16<<15 | Fault_18<<17 | Fault_19<<18 | Fault_20<<19 | Fault_21<<20 |
      Fault_23<<22 | Fault_24<<23 | Fault_25<<24 | Fault_26<<25 | Fault_27<<26 | Fault_28<<27 | Fault_29<<28 |
      Fault_30<<29 | Fault_31<<30;
  Faultec=0; 
  }
  //FAULT_Modbus1=FAULT; FAULT_Modbus2=FAULT>>16; //25_01_21
  
  //маска OutX4_4 - запрет включения 03_04_19
  Out_X4_5 = ((FAULT ) != 0); //if(FAULT & 0x004D7FEE)Out_X4_4=1; else Out_X4_4=0; //& 0x004D7FEE
  
#ifdef AFB_40
  if ((FAULT ==0) && F_Cap_Contr && (F_Cap_11 && F_Cap_12) && (Out_X4_6==0)&& ((f_Zapr_Zar)==0) && (IGCT_Test ==0)) // 18_08_21 -F_Cap_Contr 
  {
    Status_Flag |= 0x0400; Status_Flag &= 0xfdff; F_AFB_Good =1; 
  } // наличие ошибок - МодБасс  ///TEMP 26_11
  else { Status_Flag &= 0xfbff;  F_AFB_Good =0; //Status_Flag |= 0x0200; //08_10_20
#else
  if ((FAULT ==0) && F_Cap_Contr && (F_Cap_11 && F_Cap_12) && (Out_X4_6==0)&& ((Trig_Prot & 0x8000)==0) ) // 18_08_21 -F_Cap_Contr 
  {
    Status_Flag |= 0x0400; Status_Flag &= 0xfdff; F_AFB_Good =1; 
  } 
  else { Status_Flag &= 0xfbff;  F_AFB_Good =0; //Status_Flag |= 0x0200; //08_10_20
#endif
  
  if((FAULT_Modbus1&(FAULT_Modbus2<<16))!=FAULT){
	FAULT_Modbus1=FAULT; FAULT_Modbus2=FAULT>>16; Status_Flag |= 0x0800; 
  	if((F1_KQF1 == 0) && ( F2_KQF1 == 0) && (F0_QF1 == 0)) F8_KM1 = 2; }}  // отсутствие ошибок - МодБасс //21.03.24 доб-л F8_KM1 = 2;  //тест 
	FAULT_Modbus1=FAULT; FAULT_Modbus2=FAULT>>16;

  //сохранение перед выключением
  if ((Fault_16==1)&&(F_Zapis==0)){
    F_Zapis = 1;
    F_I2C_Wr = 1; // через 100мсек F_I2C_Wr = 2;
  }
    
  //превышение температуры
  static uint16_t Ch_HOT;
  if ((inp1_7 && inp1_8)==0){
    Ch_HOT++;
    if( Ch_HOT>2000 ){ Fault_19=(inp1_7==0); Fault_21=(inp1_8==0); Faultec=1;} //скорость срабатывания 250мкс (мало?) +команда вык-ть \\Otkaz_Vkl=1; 20_12_19 
  }
  else { 
    Ch_HOT=0;  Fault_19=0; Fault_21=0; FAULT &=0xffebffff;} //if(Fault_19&&(FAULT ==0))Otkaz_Vkl=0;
  //Отказ включения (27_03)
  //Otkaz_Vkl |= Fault_06; //05_01_21 по письму 29_12_20
  
  //мигание выход для контроля работы МУ AFB
  if(Ch_Puls_X4_1 > 799){Ch_Puls_X4_1=0; Out_X4_1 ^=1;} 
  Ch_Puls_X4_1++;
  
  //маска триггера защит //05_01_21
  if(((STATUS==2)||(STATUS==5))==0) //Trig_Prot |= 0x06; //29_09_20
  {
    if((Trig_Prot & 0x0080)==0) Trig_Prot |= 0x02 ; //13_01_21
    if((Trig_Prot & 0x0020)==0) Trig_Prot |= 0x04 ; //13_01_21
  }//else Trig_Prot |= 0x46;
  
  //запрет откл защиты  06_01_21
 // if((Trig_Prot&0x02)==0)
 // {Trig_Prot|=0x80;}
  // Ошибки УРОВ 11_01_21
 if(((inp4_3 || inp4_4 || inp4_8 || F_Over_Current)!=0) && ((Ch_Otkaz_Otkl_1 || Ch_Otkaz_Otkl_2)!=0))
 {
   Out_X4_6 = 1; FAULT |= 0x2000; if(Ch_Otkaz_Otkl_1)Fault_31=1; if(Ch_Otkaz_Otkl_2)Fault_30=1; Faultec=1;
 }
 
}

/////////////////////////////////////////////////////////////////////////////////
extern int16_t dI_dt_50mks;
extern uint16_t P_dIdt_Set_50mks;
extern uint16_t N_dIdt_Set_50mks;
static uint8_t Ch_P_Io; //сверхТок +
 uint8_t Ch_dIdt_PP;
 uint16_t Ch_ImaxP=0;
 uint16_t Ch_ImaxN=0;
static uint8_t Ch_Io_N;
static uint8_t Ch_dIdt_N;
//static uint8_t Ch_Otkaz_Otkl_Sbr;
//uint8_t Ch_Otkaz_Otkl; //перенес выше
//static uint8_t Ch_Strahovka;
int16_t I1_N_otkl=0;
int16_t dI_dT_1=0;
int16_t dI_dT_2=0;
extern int16_t dI_dt_50mks_F;
uint16_t Ch_Urov=0;
uint8_t Ch_F_16=0;
//uint16_t Status_Flags=0;
 extern  USHORT   usRegInputBuf[REG_INPUT_NREGS];
 extern uint8_t f_Low_24V;
 extern uint8_t C12_4;
 int16_t XxX3 =0;

void Proverka_Zashit(void) //50 mks
{
  //проверка защит - выбор характеристики отключения
  if(((F_Over_Current==0)&&(F2_KQF1==0))&& (f_Blok ==0)) //&&(FAULT==0) 21_07_20 &&(F1_KQF1==0)//16_06_20 +22_06 ->f_Blok
  {
    Prichina_Otkl &= (~0x01);
    //XxX3 = I1_N;
  //для прямого тока отсечка 
      if ((I1_N > (int16_t)P_Io_Set )&&((Trig_Prot & 0x02)!=0))//|| I1_N > 10000
      {
        Ch_P_Io++;
        if( Ch_P_Io > 1 )
        { 
          F_Over_Current=1; //Otkaz_Vkl=1;скорость сраб-я 100мкс (мало?)
          //Status_Flag &=0xff40; //07_08_20
          Status_Flag |=0x0002;//Funk_Otkl=1; 
          Zapis_Sobitij (0x01);
        } 
      }
      else Ch_P_Io=0;
    

  //для прямого тока dI_dt
    if (((I1_N >(int16_t)1000)&&(dI_dt_50mks > (int16_t)P_dIdt_Set_50mks) // 11_01_19            //
        || (dI_dt_50mks_F > (int16_t)P_dIdt_Set_50mks))&&((Trig_Prot & 0x08)!=0)) //нарастание тока+ I1_N < 3000//& (I1_N_1_6ms < 3000)//16_06_20 -&& ((SPI2_Priem & 0x4000) ==0) 
    {
      Ch_dIdt_PP++;
      if( Ch_dIdt_PP > 0x02 )
      { 
        F_Over_Current=1; 
        //Status_Flag &=0xff00; //07_08_20
        Status_Flag |=0x0008; //Funk_Otkl=3;
        Zapis_Sobitij (0x02); //
      } //Otkaz_Vkl=1;скорость срабатывания 250мкс 
    }
    else 
      Ch_dIdt_PP=0;
    
 // для прямого тока Imax+
    if(((int16_t)I1_N_1_6ms >(int16_t)P_Imax_Set)&&((Trig_Prot&0x80)!=0))
    {
      Ch_ImaxP ++; 
      if((Ch_ImaxP+11) > (P_Imax_TSet*20))
      {
        F_Over_Current=1; 
        //Status_Flag &=0xff40; //07_08_20
        Status_Flag |=0x0080; //
        Zapis_Sobitij (0x09); //
      }
    }
    else Ch_ImaxP = 0;
    
  //для обратного тока отсечка - блокировка защиты "&& (SPI2_Priem & 0x4000 ==0)" письмо 04_09_19 
    if ( ((int16_t)I1_N < (int16_t)(0-N_Io_Set ))&&((Trig_Prot&0x04)!=0))//сверхТок - && (SPI2_Priem & 0x4000 ==0) возвращаю 12_06_20 //16_06_20 -&& ((SPI2_Priem & 0x4000)
    {
      Ch_Io_N++;
      if( Ch_Io_N>1 )
      { 
        F_Over_Current=1; 
        //Status_Flag &=0xff40; //07_08_20
        Status_Flag |=0x0004;//Funk_Otkl=4;
        Zapis_Sobitij (0x03);
      } //Otkaz_Vkl=1;скорость срабатывания 100мкс 
    }
    else Ch_Io_N=0;
  
  //для обратного тока dI_dt - блокировка "&& (SPI2_Priem & 0x4000 ==0)" письмо 04_09_19 
    if ((((int16_t)dI_dt_50mks < (int16_t)(0-N_dIdt_Set_50mks)) && ((int16_t)I1_N <= (int16_t)-1000)            //
        || (((int16_t)dI_dt_50mks_F < (int16_t)(0-N_dIdt_Set_50mks)) && ((int16_t)I1_N <= (int16_t)0)))
          && ((Trig_Prot&0x10)!=0) ) //нарастание тока- //&& (SPI2_Priem & 0x4000 ==0) возвращаю 12_06_20 //16_06_20 -&& ((SPI2_Priem & 0x4000)
    {
      //dI_dT_1 = dI_dt_50mks;
      dI_dT_2 = (0-N_dIdt_Set_50mks);
      Ch_dIdt_N++;
      if( Ch_dIdt_N>2 )
      { 
        F_Over_Current=1; 
        //Status_Flag &=0xff40; //07_08_20
        Status_Flag |=0x0010;//Funk_Otkl=5;
        Zapis_Sobitij (0x04);
      } //Otkaz_Vkl=1; скорость срабатывания 250мкс 
    }
    else Ch_dIdt_N=0;
    
     //для обратного тока Imax-  Trig_Prot&0x40;
    int16_t Temp_Imax =0;
    Temp_Imax = ((int16_t)I1_N_1_6ms < (int16_t)(~(int16_t)N_Imax_Set));
    if(Temp_Imax && ((Trig_Prot & 0x20)!=0))
    {
      Ch_ImaxN++; 
      if((Ch_ImaxN+11) > (N_Imax_TSet*20)){
        F_Over_Current=1; 
        //Status_Flag &=0xff40; //07_08_20
        Status_Flag |=0x0100; //
        Zapis_Sobitij (0x0a); //
      }
    }
    else Ch_ImaxN = 0;
    
  }
  
  //Запрет отключения  //Запрет отключения  //Запрет отключения  //Запрет отключения  //Запрет отключения ////16_06_20 - СШ сказал УРОВ по отриц току убрать

  #ifdef AFB_40
   int16_t I1_Purov = 12000;  //20_11_20 //30_11_22 
   int16_t I1_Nurov = -10000; //20_11_20
   int16_t dI_dt_PUr = 1000;
   int16_t dI_dt_NUr = -250;
   if((STATUS==1)||(STATUS==4)){ 
      I1_Purov = 10000;   //20_11_20
      I1_Nurov = -10000;  //20_11_20
      dI_dt_PUr = 1000;   //20_11_20
      dI_dt_NUr = -250; } //20_11_20
  #else
   int16_t I1_Purov = 10000; //30_11_22 /12_12_22
   int16_t I1_Nurov = -5000; //20_11_20 /12_12_22 
   int16_t dI_dt_PUr = 1000;
   int16_t dI_dt_NUr = -150;//12_12_22 
   if((STATUS==1)||(STATUS==4)){ 
      I1_Purov = 4500;
      I1_Nurov = -4500;
      dI_dt_PUr = 150;    //20_11_20//12_12_22 
      dI_dt_NUr = -150; } //20_11_20//12_12_22 
  #endif   

  // I1_N = -5000; //темп
   //по мгн. току
    if( ((F2_KQF1==0) && (f_Blok == 0)) 
       && (((I1_N > I1_Purov))
       ||((I1_N < I1_Nurov)))  ) //10_06  //&& (SPI2_Priem & 0x4000 ==0) возвращаю 12_06_20 //
    {   
       //dI_dT_1 = T_Otkl;       // TEMP // TEMP //
       //I1_N_otkl = dI_dt_50mks; // TEMP // TEMP //
       //Ch_Otkaz_Otkl_Sbr =0;
       if (Ch_Otkaz_Otkl_1 < 2)Ch_Otkaz_Otkl_1 ++;
       if((Ch_Otkaz_Otkl_1 > 1)&&(Fault_31==0)&& (((I1_N > I1_Purov)&&(Trig_Prot&0x0082))||((I1_N < I1_Nurov)&&(Trig_Prot&0x0024)))) { // 06_01_21 //27_04_21
         Fault_31=1; Faultec=1;
         Zapis_Sobitij (0x05); 
         Status_Flag |=0x0020; //
         Out_X4_6=1;
      }
     
    }
   else
   {
     //if((F2_KQF1||f_Blok)==0) 
       Ch_Otkaz_Otkl_1=0;
   }
   
   //по скор. нарастания
    if( ((F2_KQF1||f_Blok) == 0) 
       && (((dI_dt_50mks > dI_dt_PUr)) 
       || (((dI_dt_50mks < dI_dt_NUr)&&(I1_N < (int16_t)(-150))))) ) //10_06  //&& (SPI2_Priem & 0x4000 ==0) возвращаю 12_06_20 //
    { 
       //Ch_Otkaz_Otkl_Sbr =0;
       if (Ch_Otkaz_Otkl_2 < 2) Ch_Otkaz_Otkl_2 ++;
       if((Ch_Otkaz_Otkl_2 > 1)&&(Fault_30==0)&& (((dI_dt_50mks > dI_dt_PUr)&&(Trig_Prot&0x0008)) //27_04_21
       || (((dI_dt_50mks < dI_dt_NUr)&&(I1_N < (int16_t)(-150)))&&(Trig_Prot&0x0010)))) {  //27_04_21
         Fault_30=1; Faultec=1;
         Zapis_Sobitij (0x05); 
         Status_Flag |=0x0020; //
         Out_X4_6=1;
      }
    }
   else //по писму СШ 19_11_20
    { 
      //Ch_Otkaz_Otkl_1=0; 
      //if((F2_KQF1||f_Blok)==0) 
        Ch_Otkaz_Otkl_2=0;
      /*if (FAULT & 0x00800000)//Fault_24 
      { 
        if(Ch_Otkaz_Otkl_Sbr > 8) { if((FAULT & 0x00480500)==0){ Fault_24=0; FAULT &= 0xFF7FFFFF; }  }  
        Ch_Otkaz_Otkl_Sbr++;
      } */
      //if(Out_X4_6 && ((FAULT & 0x00480500)==0)){Ch_Urov--; if(Ch_Urov==0)Out_X4_6=0;}
      //else Ch_Urov = 400;
    }
    //if ((F_Cap_11==0)||(F_Cap_12==0)){}}
   
       //перенос в проверку защит 01_04_21
    if (f_Low_24V && (f_Zapusk==0)){ 
      Ch_F_16++; 
      if (Ch_F_16>8) { 
        if((Fault_16==0)&&((inp1_1==1)&&(inp1_2==0)&&(inp1_3==1)&&(inp1_4==0))) 
          C12_4 = 1; //
        Fault_16 =1; Faultec = 1;
      } 
    } 
    else Ch_F_16=0;
   
}


void Zapis_Sobitij (uint8_t Sobitie)
{
  i_BS++; 
  if (i_BS > 99) i_BS=0;
  //Buff_Sobitij [i_BS]= Sobitie; //Буффер событий 
  usRegInputBuf[i_BS]= Sobitie;
  usRegInputBuf[101]= i_BS;
}

////////////////////////////////////////////////////////////////////////////////

uint16_t Ch_Rozr = 0;   uint8_t  Ch_Wr_2 = 0;
uint8_t f_Rozr = 0;
extern uint8_t C12_4;
uint16_t Ch_C12_4 = 0;

void Rozriad_C12 (void) //10ms 
{
  if(Vkl_VS1_2)
  {
    //F8_Proverka=2; //подумать
    if(((inp1_6 && inp1_5)==0)) {F8_KM1 = 1; F8_KM2 = 1; Ch_Rozr = 0;} //(Vkl_VS1_2 ==1)&& 27_01_20 //20_03_24
    if((Vkl_VS1_2 ==1)&&((inp1_6 && inp1_5 && inp1_3 && inp1_1) == 1)){ //&&((Trig_Prot & 0x8000)!=0)
		f_Rozr = 1; Vkl_VS1_2 = 2;}
    //if ((V1_N < 100)&&(V1_N > (-100))) { F8_KM1 = 2; F8_KM2 = 2; Vkl_VS1_2 =0; Ch_Rozr =0;} //&& (V2_N < 40)
    if ((Ch_Rozr > 300)) {  
       F_Cap_Contr = 0;
       if ((V1_N > 100)||(V1_N < (-100)))FAULT |= 0x02000000; //ошибка_26
	F8_KM1 = 2; F8_KM2 = 2; Vkl_VS1_2 = 0; Ch_Rozr = 0;} //(Vkl_VS1_2 ==2)&&ошибка_26 F8_KM1 = 2; F8_KM2 = 2;
    if((inp1_6 && inp1_5)==1) Ch_Rozr ++; 
  }
  
  #ifdef AFB_40
  //31_03_21 розряд С12 для 4-ки
  if(C12_4) {
      if(C12_4==1) {Ch_IGCT =30; C12_4 =2;  Ch_C12_4 =0;} // 24_12_21
      if(C12_4==2) {Ch_C12_4 ++; if(Ch_C12_4 >30){if((V1_N > 100)||(V1_N < (-100)))
        FAULT |= 0x08000000; C12_4=0; };}//ошибка_28
  }
  #endif
  
  ////запись номера  //12_04_24
  if( F_Zapis ) {
	if(Ch_Wr_2 >9) F_I2C_Wr = 2;  
	Ch_Wr_2 ++;
  }
  
  
}

////////////////////////////////////////////////////////////////////////////////

uint8_t Ch_JgCT =0;
uint8_t Ch_JgCT_CZ =0;
extern int16_t I2_N;

void IGCT_Contr (void) //-10мсек /по письму 18_09_20
{
  #ifdef AFB_40 
  if(((Ch_IGCT | IGCT_Test) == 0)&&((F2_KQF1)==0))  // || F1_KQF1
  {
    if((Opto_IN2 != 0)) 
    { 
      if(Ch_JgCT > 4){
        FAULT |= 0x01; //FAULT |= Opto_IN2;
        if((I2_N > 80)&&((inp1_3 == 0)&&(inp1_4 == 1))) {
          Out_X4_6 = 1; FAULT |= (0x01<<19); Fault_20 = 1;} //(V1_N < 100)
        //if(Opto_IN3) {Fault_26 = 1; Out_X4_6 = 1; FAULT |= (0x01<<25);}
      }
      else Ch_JgCT ++;
    }
    else Ch_JgCT =0;
  }
  
  //29_03_21 - V1_N < 100 -заменил на I2_N > 80 по желанию СШ
  if ( (Opto_IN2 != 0)&&(inp1_3 == 1)&&(inp1_4 == 0)&&(inp1_1 == 0)&&(inp1_2 == 1)
      &&(V1_N<(int16_t)860 ||((FAULT&0x08)!=0))&& F_Cap_Contr ) // F_Cap_Contr (Ch_IGCT==0) 09_06_21
  { 
    if(Ch_JgCT_CZ > 5){ //было 0 09_06_21
    //if ((F2_KQF1!=0)&&(T_Otkl > 580)) {
      if(I2_N > 80) {Out_X4_6 = 1; FAULT |= (0x01<<19); Fault_20=1;}
      FAULT |= 0x01; //}
      //if(Opto_IN3) { //10_06_21
        //Fault_26 = 1; Out_X4_6 = 1; FAULT |= (0x01<<25);}
   // else {
   //   if(I2_N > 80) {Out_X4_6 = 1; FAULT |= (0x01<<19); Fault_20=1;}
   //   FAULT |= 0x01;}
    }
    else Ch_JgCT_CZ ++;
  }
  else Ch_JgCT_CZ =0;
  #endif
}


//////// The end ///////