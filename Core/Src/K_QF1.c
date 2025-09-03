
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

#include "Algoritm.h"
//#include "Tiristor.h"
#include "K_QF1.h"
#include "Modbus.h"

/* ------ variables ---------------------------------------------------------*/
extern volatile int16_t I1_N;
extern uint8_t Out_X1_1, Out_X1_2;      //Управление QS1
//extern uint16_t Input_X1;
extern uint8_t Opto_IN1;
extern uint8_t Opto_IN2;
extern uint8_t Opto_IN3;
extern uint8_t F_Cap_11 ;           //Состояние заряда конденсатора С1:1   -> 0-не заряжен, 1-заряжен 840-1100
extern uint8_t F_Cap_12 ;           //Состояние заряда конденсатора С1:2   -> 0-не заряжен, 1-заряжен 965-990
extern uint8_t F_VS2_Good ;         //Результат испытания (при включ-е) работы тиристора VS2
extern uint8_t Fault_25 ;         //Результат испытания (при включ-е) исправности диода VD2  (Opto_IN1)
extern uint8_t F_AFB_Good ;         //Результат проверки AFB: 1- готов; 0- неготов или ошибка.
extern uint8_t F8_KM1 ;
extern uint8_t F8_KM2 ; 
extern uint8_t F8_QS1 ; 
extern uint8_t F0_QS1 ;

//extern int16_t I1_N_1_6ms; // 1.6мс или 32 измерений через 0,05мс каждое 
extern int16_t I1_N_64ms;// 64мс или 128 измерений через 0,5мс каждое
extern int16_t I2_N_64ms; //27_08_20
//extern uint16_t Count_Switch ;
//extern  USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]; //

extern uint8_t Fault_09;
extern uint8_t Fault_11;
extern uint8_t Fault_12;
extern uint8_t Fault_23;
extern uint8_t Fault_24;
//uint8_t Fault_01=0; // TEMP
uint8_t Fault_13=0;
uint8_t Fault_17=0;
uint8_t Fault_18=0;
uint8_t Fault_26=0;
uint8_t Fault_28=0;
extern uint32_t FAULT;

 extern uint16_t LTD_Threshold; 	// 02_09_24
// extern uint16_t LTD_Izmer; 		// 02_09_24
extern uint8_t f_Test;
uint8_t F8_Proverka=0;
int32_t V3_TEMP;
int16_t I2_Ntemp =0;

/*------------*/
/*-------------------------Управление Выключателем QF1------------------------*/
#define TT1_QF1 20000     //Заданный интервал в циклах для контроля времени включения                   /было 20000
#define TT2_QF1 3000     //Заданный интервал в циклах для контроля времени отключения                  /было 10000
#define TT3_QF1 10000    //Заданный интервал в циклах для контроля времени неопределённого состояния    /было 10000

uint8_t F0_QF1 = 0;     //Состояние QF1: 0 - отключен 1 - включен 2 - неопределенное состояние
uint8_t F1_QF1 = 0;     //Флаг управления циклом включения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F2_QF1 = 0;     //Флаг управления циклом отключения: 0 - цикл не запущен 1 - цикл выполняется 2 - цикл завершен
uint8_t F8_QF1 = 0;     //Внешнее управление (команда): 0 - нет команды 1 - включить 2 - отключить
uint8_t F9_QF1 = 0;     //Копия F8_QF1 (для цепочных команд)

uint16_t T1_QF1 = 0;    //Счетчик циклов при включении
uint16_t T2_QF1 = 0;    //Счетчик циклов при отключении
uint16_t T3_QF1 = 0;    //Счетчик циклов при неопределённом состоянии
//uint16_t CT1_QF1;       //Счетчик включений/выкл-й
uint16_t T4_QF1 = 0; 	//проверка на реле //проверка на реле //проверка на реле //проверка на реле //проверка на реле //проверка на реле 
uint16_t T5_QF1 = 0; 	//Счетчик задержки для формир-я импульса включения QF1 

//Входы-Состояние 
#define inp1_1 ((Input_X1 & 0x0001)>>0) //QS1 отключен
#define inp1_2 ((Input_X1 & 0x0002)>>1) //QS1 включен
#define inp1_3 ((Input_X1 & 0x0004)>>2) //БВ отключен
#define inp1_4 ((Input_X1 & 0x0008)>>3) //БВ включен
#define inp1_5 ((Input_X1 & 0x0010)>>4) //KM1 включен
#define inp1_6 ((Input_X1 & 0x0020)>>5) //KM2 включен

extern uint8_t Out_X1_3;     //Управление QF1 - Включить
extern uint8_t Out_X1_7;     //тест на реле Управление QF1 - Выключить ///////////////проверка на реле //проверка на реле //проверка на реле 
extern uint8_t Out_X4_6;
extern uint8_t Gash_Duge; 	
extern uint8_t Funk_Otkl;
extern uint16_t dI_dt_150mks;
extern uint8_t Faultec;
extern uint8_t F_Cap_Contr;

int16_t Ch_F_11 = 0;	//счётчик ошибки №11	
//int16_t XxX1 = 0; //temp
//int16_t qwerty = 0; //temp

void QF1_Control(void) //50мкс
{
  F9_QF1 = F8_QF1;
  if ((F9_QF1==1)&&(F1_QF1==0)&&(F2_QF1==0))
  {
    F1_QF1 = 1;
    Out_X1_3 = 1;
    T1_QF1 = TT1_QF1;
    T2_QF1 = 0;
    T4_QF1 = 16000;
    T5_QF1 = 30000;  //увеличил 15_03_20
  }
	
  if (F1_QF1 == 1) 	//?
  { //Out_X1_7=0; 	//проверка на реле //проверка на реле  TEMP
	if (T1_QF1 > 0)
	{
	  T1_QF1 --;
	}
	else 			//счёт закончен
	{
	  if ((inp1_3==0)&&(inp1_4==1))	//БВ вкл.
	  { //T4_QF1=TT3_QF1; // задержка
		//Out_X1_3=0;
		//printf("Выключатель QF1 мех. включен \n");
	  }
	  else
	  {
		//Out_X1_3=0;
		F1_QF1 = 0;
		Fault_09 = 1; Faultec = 1; //printf("Выключатель QF1 мех. неисправен \n"); Out_X4_6=1;
	  }
	}
  }
  
  if(Out_X1_3) { T5_QF1 --; if(T5_QF1 < 1) Out_X1_3 = 0; } //формир-я импульса включения QF1 
  
  
  if ((F9_QF1 == 2) && ((F2_QF1 || Gash_Duge) == 0)) // ((F9_QF1==2)&&((F2_QF1==0) && (Gash_Duge==0))
  { //Out_X1_3 = 0; //костыль помог
    F_Cap_Contr = 0;
    F1_QF1 = 0;
    F2_QF1 = 1;
    //qwerty=I1_N;	//XxX1 = (int16_t)dI_dt_150mks;//temp		
    
    //выбор характеристики отключения
    if ((I1_N > (int16_t)(-500))) //&&(dI_dt_150mks < 1500)
      Gash_Duge = 1; 
    if ((I1_N > (int16_t)3000)&&((int16_t)dI_dt_150mks > (int16_t)300)) 
	  Gash_Duge = 2;
    if ((I1_N > (int16_t)(-500))&&((int16_t)dI_dt_150mks > (int16_t)1499)&&((dI_dt_150mks & 0x8000)!=0x8000)){
      //qwerty = dI_dt_150mks; //TEMP 4_01
	  Gash_Duge = 3; }
    if ((I1_N < (int16_t)(-499))&&(I1_N > (int16_t)(-3000))) 
	  Gash_Duge = 4;
    if ((I1_N < (int16_t)(-2999))) 
	  Gash_Duge = 5;
    if (f_Test == 1) 	//02_03_20 для тестового режима
	  Gash_Duge = 6;   
    //else Gash_Duge = Funk_Otkl; //Formirovanie_Impulsov_Otkluceniya_BV(); /////////////////////////////
    Out_X1_7 = 1; 		//проверка на реле //проверка на реле //проверка на реле TEMP
    T4_QF1 = 10000; 	//проверка на реле //проверка на реле //проверка на реле  TEMP  0,5 s
    T2_QF1 = TT2_QF1;
    T1_QF1 = 0;
  }
	
  if(T4_QF1 != 0)
      T4_QF1 --; 		//проверка на реле //проверка на реле //проверка на реле TEMP
  else {  T1_QF1 = 0; F1_QF1 = 0;  } //CT1_QF1 ++;
      //Out_X1_7=0; Out_X1_3=0;//проверка на реле //проверка на реле //проверка на реле TEMP //Out_X1_7=0; заменил на Out_X1_3=0;
  
  if (F2_QF1 != 0)
  {
    if ((inp1_3 == 1)&&(inp1_4 == 0))////БВ откл.
    {
      //printf("Выключатель QF1 мех. отключен \n");
      T2_QF1 = 0;
      F2_QF1 = 0;
      Count_Switch ++; //СТ1_QF1
      //Gash_Duge=0;
      //Out_X1_7=0;//не успевает, задержку добавить проверка на реле //проверка на реле //проверка на реле  TEMP
    }
    else
    {
      if (T2_QF1 > 0)
      {
        T2_QF1 --;
      }
      else
      {
        F2_QF1 = 0;
        FAULT |= 0x0400; //Fault_11=1; Faultec=1; //printf("Выключатель QF1 мех. неисправен \n"); //
        //Gash_Duge=0; //?//Out_X1_7=0;//проверка на реле //проверка на реле //проверка на реле  TEMP
      }
    }
  }
  

//контроль положения БВ
  if (inp1_3 == 1)
  {
    if (inp1_4 == 0)
    {
    	F0_QF1 = 0; T3_QF1 = 0;
    }
    else
    {
     	F0_QF1 = 2; T3_QF1 ++;
    }
  }
  else
  {
    if (inp1_4 == 1)
    {
    	F0_QF1 = 1; T3_QF1 = 0;
    }
    else
    {
    	F0_QF1 = 2; T3_QF1 ++;
    }
  }
  F8_QF1 = 0;
  F9_QF1 = 0;
  
  if ((F0_QF1 == 2)&&(T3_QF1 > TT3_QF1))
  {
    FAULT |= 0x0400;	//29_03  Fault_11=1;        было -- Fault_18=1; Faultec=1;//Контроль сигнала положения (оба сигнала одинаковы) 
  }
  //самосброс Fault_11 
/*  if(((FAULT & 0x0400)!=0) && (inp1_4 != inp1_3)) {
    Ch_F_11 --; //if (Ch_F_11==0) FAULT &= 0xFFFFFBFF; /18_12_19 TEMP
  }
  else Ch_F_11 = 10000;		*/
}



/*------------------------- Проверка при включение БВ ------------------------*/
uint8_t F0_Proverka = 0 ;
extern int16_t V1_N, V2_N, V3_N;
uint8_t F6_Faza = 0;     		//Флаг смены фазы включения
int16_t V1_N_Vkl = 0;
uint8_t FAZA_Vkl = 0;
uint16_t Ch_Pr = 0;
extern uint8_t F1_KQF1;     	//Флаг упр-я циклом вкл-я: 0 - не запущен 1 - выполняется 2 - завершен
extern uint8_t F_Cap_Contr;

void Proverka_Pered_Vkl(void) 	//50mks
{
  if (F8_Proverka)
  {
    if ( F8_Proverka == 1 ) {	// Pervij_Prohod==0
      F_VS2_Good = Opto_IN1; 	//VS2 -не пробит 
      F0_Proverka = 0; F_Cap_11 = 0; F_Cap_12 = 0;  //
      if(F_VS2_Good) F8_Proverka = 2;  //запуск формирования пачки импульсов (8мсек)- если БВ разомкнут
      else {
        F8_Proverka = 0; F1_KQF1 = 0; Fault_24 = 1; Faultec = 1;}
      #ifdef AFB_40
        F8_KM2 = 2; //22_05_20 temp КМ2 по идее отключен
      #endif
    }
    
#ifdef AFB_40 
    
    if (F8_Proverka == 7) {  
      if ((V1_N > (int16_t)870) && (V2_N > (int16_t)980)) { //проверка заряда конд-в
        FAZA_Vkl ++; F6_Faza = 1; F8_Proverka = 0; 
        Fault_24 = (F_VS2_Good == 0);  //if( I1_N_1_6ms<(-10) ) Fault_13 = 1; //||(Opto_IN2)-//22_05_20 //23_11_20
        //FAULT |= ((Opto_IN2)<<4); //22_05_20
        Fault_17 = ( (int16_t)I1_N_64ms > 120 ) || ( (int16_t)I1_N_64ms < (-120) );
        Faultec |= Fault_13||Fault_17 || Fault_24;
        F0_Proverka = ( F_VS2_Good  && ((Fault_25 || Faultec || Opto_IN2) == 0) );
      }
      else {
        F8_Proverka = 0; Fault_13 = 1; F8_KM1 = 2; Faultec = 1; F1_KQF1 = 0; //Fault_26 = Opto_IN3; //10_06_21
      }
    } 
    
//проверка тиристоров гашения 29_03_21    
    if ((F8_Proverka == 4) ) { //1 команда вкл. КМ2  && (inp1_6==0
      if(inp1_6 == 0) {F8_KM2 = 1; Ch_Pr = 0;}
      else {
        if (Ch_Pr > 600) { 
          F_VS2_Good = Opto_IN1; Fault_24 = (F_VS2_Good == 0); 
          F8_Proverka = 7; Ch_Pr =0; Faultec = 1;} //F8_Proverka = 0; Fault_13 = 1; Faultec = 1;F1_KQF1 = 0; 
        Ch_Pr ++;
      }
    } 
    
    if ( F8_Proverka == 3 ) {//ожидаем завершения пачки имп-в
      //Fault_13 = (((int16_t) I1_N_1_6ms < (-100))) ;  //||(Opto_IN3)- 22_05_20  //||(F_VS2_Good == 0)23_11_20
      //Faultec |= Fault_13;  //23_11_20
      //F_VS2_Good = Opto_IN1; //VS2 -не пробит 
      //Fault_24 = (F_VS2_Good == 0); Faultec = (F_VS2_Good == 0);
      if((FAULT || Faultec)==0)F8_Proverka = 4; // F8_Proverka = 4;
      else { F8_Proverka = 0;  F1_KQF1 = 0; }
    }
    
#else
    if ( F8_Proverka == 3 ) {//ожидаем завершения пачки имп-в
      //F_VS2_Good = Opto_IN1; //VS2 -не пробит 
      //Fault_24 = (F_VS2_Good == 0);//if( I1_N_1_6ms<(-10) ) Fault_13 = 1; //23_11_20
      //Fault_13 = ((int16_t) I1_N_1_6ms < (-100)); //23_11_20 //СШ не хочет контроллировать ток 15_03_21
      Faultec |= Fault_13 || Fault_24; //23_11_20
      if((FAULT || Faultec) == 0)F8_Proverka = 4; 
      else {F1_KQF1 = 0; F8_Proverka = 0; }
    }
    
    //Проверка резистора ИКЗ при включение (inp1_6==0)
    if ( F8_Proverka == 4 ) { //1 команда вкл. КМ2  && (inp1_6==0
      if(inp1_6 == 0) {F8_KM2 = 1; Ch_Pr = 0;}
      else {
        if (Ch_Pr > 600) { 
          F_VS2_Good = Opto_IN1; Fault_24 = (F_VS2_Good == 0); 
          F8_Proverka = 5; Ch_Pr = 0; Faultec = 1;} //F8_Proverka = 0; Fault_13 = 1; Faultec = 1;F1_KQF1 = 0; 
        Ch_Pr ++;
      }
    } 
    
    if ( F8_Proverka == 5 ) {  //18_03_21
      if ((V1_N > 860)&&(V2_N > 980)&&(Opto_IN1)){ //2 когда вкл-но      && F_Vkl_Prov==1
           F8_Proverka = 6;  // VS2 на 10мсек //Gash_Duge=7;
		   V1_N_Vkl = V1_N; 
           F8_KM2 = 2;  	// TEMP время разряда = скорость выключения КМ2 ???
           F_Cap_Contr = 0; //14_04_21
        }
      else { 
        F8_Proverka = 0; 
        if ((V1_N < 860)||(V2_N < 980)) { Fault_13 = 1;  F8_KM1 = 2; }
        if(Opto_IN1 == 0) Fault_24 = 1; Faultec = 1; F1_KQF1 = 0; }
    }
      
    if ( F8_Proverka == 7 ){               //3 оценка результата 
      if (( V1_N_Vkl - V1_N ) < (int16_t)20) { //(V1_N+15) > V1_N_Vkl 
        Fault_28 = 1;
        FAULT |= 0x01 << 27;}
        F8_Proverka = 8;
      //FAULT =0; FAULT |= Fault_28<<27;//FAULT &=0x8000000;//Fault_02=0; Fault_03=0; 
    }
      
    if ((F8_Proverka == 8) && (V1_N > (int16_t)870) && (V2_N > (int16_t)980) ) { //4 ожидание заряда конд-в
        //FAULT &=0x8000000; F_Vkl_Prov=5;  Fault_02=0; Fault_03=0; ////не проверять в процессе вкл-я Ch_F_Cap=0;  13_11
        FAZA_Vkl ++; F6_Faza = 1; F8_Proverka = 0; F0_Proverka = ( F_VS2_Good  && (Fault_25 == 0));
    } 
#endif 

  } 
}



/*------------------------- Процесс отключения и включения БВ ------------------------*/

extern uint8_t F1_KQF1;     //Флаг упр-я циклом вкл-я: 0 - не запущен 1 - выполняется 2 - завершен
extern uint8_t F2_KQF1;     //Флаг упр-я циклом откл-я: 0 - не запущен 1 - выполняется 2 - завершен
uint8_t F5_Faza = 0;     //Флаг смены фазы отключения

uint8_t F_Napravl_I = 0;     //Флаг направления тока осн. цепи 0-прямое, 1-обратное
uint8_t FAZA_Otkl = 0;
uint32_t FAULT_O = 0; //ошибки после отключения БВ, запись в ППЗУ и Модбас

uint16_t Sum_I1 = 0; //Cумма токов для усреднения
uint16_t T_Otkl = 0; //счётчик режима отключения
uint16_t T_Vkl = 0; //счётчик режима включения
#ifdef AFB_40   // {200,4,20,300,350,400,540,670,870,11000}; 27_08_20 
 uint16_t T_Faza_O [ ] = {200,4, 20,300,350,400,540,2400,2600,2800,12800}; // время переходов между фазами откл-я в мсек *2 было -> uint16_t T_Faza_O [ ] = {200,4,500,540,600,690,830,960,2090,10090,};
#else                  //  1  2  3   4   5   6   7   8    9    10    11   12
 uint16_t T_Faza_O [ ] = {200,4,300,340,400,490,830,1000,1030,2670,10690,11690}; // время переходов между фазами откл-я в мсек *2 //увеличил на 250мсек
#endif  
uint16_t T_Faza_V [ ] = {200,20000,10000,10260,11400,11700,14000}; /// время переходов между фазами вкл-я в мсек *2  {200,20000,10000,10260,11850,12100,14100}
//200,20000,10000,10260,11300,11600,13900
uint8_t F_TA2_Con=0; //Флаг проверки ТА2 
int32_t SumI2_N=0;
extern int16_t I2_N;
//extern uint16_t V1_N;
uint8_t Otkaz_Vkl =0;   //Флаг отказа включения
extern uint8_t Fault_07;
extern uint8_t Fault_15;
extern uint8_t Fault_20;
extern uint8_t Fault_30;
extern uint8_t F8_Pr_IKZ;
extern uint8_t F0_Pr_IKZ;
extern uint8_t f_Zapusk;
extern uint8_t f_Low_24V;
uint8_t Otkaz_IKZ=0;
extern uint16_t LTD_Izmer;
//extern int16_t I1_Otkl; //ток перед отключением
extern uint8_t BV_Otkl; //27_08_20
extern uint8_t f_QS_f12; //17_03_21
extern uint8_t IGCT_Test; //9_04_21
extern uint8_t F_Cap_Contr;
extern uint16_t f_Zapr_Zar;
//int16_t I2_Otkl = 0;
//int16_t I3_Otkl = 0;
//int16_t I4_Otkl = 0;
//int32_t II_Otkl = 0; //32-?
uint8_t Vkl_VS1_2=0;
uint8_t C12_4=0; //флаг разряд конд-ра для 4-ки
#define inp4_5 ((Input_X2 & 0x0010)>>4) //разреш. на вкл.
#define I_Gashen_max  (int16_t)135 //максимальный ток, для проверки процесса гашения
extern uint8_t Sled_Wr; //13_03_25

#ifdef AFB_40 
  uint16_t Ch_IGCT = 0;
#endif 
  
 uint8_t  TEMP_KM =0;

void BV_Process (void) //500 mks
{
  // процесс отключения //// процесс отключения //// процесс отключения //
  if (F2_KQF1 != 0)
  {
    
    if( F5_Faza == 1 )// первый проход после смены фазы
    {
      F5_Faza = 0;  
      switch (FAZA_Otkl)
      {
      case 0: break;
      case 1: break; //перенести в Algoritm (void)
      case 2: if(((int16_t)I1_N_1_6ms > I_Gashen_max)||((int16_t)I1_N_1_6ms < (-I_Gashen_max))){   //1,5 мсек
              Otkaz_Vkl = 1; Fault_23 = 1; Faultec=1; Out_X4_6 = 1;}  //printf("сбой гашения дуги \n");Gashen_Control();
              I2_Otkl = I1_N; //сохранение значение тока
           #ifdef AFB_40
              F8_KM1 = 2; F8_KM2 = 2; //13_04_21
           #endif
              break;
			  
      case 3: 
           #ifdef AFB_40 //10мс
              if((Opto_IN2==0) && (((int16_t)I1_Otkl < 200)&&((int16_t)I1_Otkl > (-200)))) {Ch_IGCT = 20; IGCT_Test = 1; }  //на 10-й мсек включение IGCT на 5мсек если ток меньше +/-200А 17_06_21 - 5мсек поменял на 10мсек
              //else {
              //  FAULT |= Opto_IN2; Out_X4_6 |= Opto_IN2; } //14_07_20 -26_08_20 -04_07_22
           #else
              if ((inp1_3 == 0)||(inp1_4 == 1)){ //250мсек
                Fault_09=1; Faultec=1;  if ((inp1_3 == 0)&&(inp1_4 == 1))Out_X4_6 = 1; }  //printf("сбой механич. разм. \n");проверка сост-я блок-контактов Otkaz_Vkl = 1; 06_01_21
           #endif
			break;	   
      case 4: 
           #ifdef AFB_40 //150мс
               if ((inp1_3 == 0)||(inp1_4 == 1)){ //150мсек
                Fault_09=1; Faultec=1; if((inp1_3 == 0)&&(inp1_4 == 1)) Out_X4_6 = 1; } break; //Otkaz_Vkl = 1; printf("сбой механич. разм. \n");проверка сост-я блок-контактов
           #else
               if ( (int16_t)I2_N > (int16_t)0 ) { F8_KM1=2; F_Napravl_I = 0;} // или I2_N  //270мсек
               else { F8_KM2 = 2; F_Napravl_I = 1; } break;
           #endif

      case 5: if ( F_Napravl_I ) { F8_KM1 = 2; }   //200мсек /175мс
            else { F8_KM2 = 2; } break;     
  
      case 6: //F_TA2_Con=1; SumI2_N=0;                  //245 - надо 200  /мсек
              if (( (int16_t)I1_N_64ms > I_Gashen_max )||((int16_t)I1_N_64ms < (-I_Gashen_max)))
              { Otkaz_Vkl = 1; Fault_23 = 1; Faultec = 1; Out_X4_6 = 1;} I3_Otkl = I1_N; break; //printf("сбой гашения дуги 2 \n");&& ( inp1_4 || inp1_5 || inp1_6)
      
      case 7: //F_TA2_Con=0;                                     //40-> 270 25-> 415мсек
            #ifdef AFB_40 //08_10_21 //270мс
               if (((int16_t)I2_N_64ms > 80 )||((int16_t)I2_N_64ms < (-80))) { // if ( (SumI2_N > 6000)||(SumI2_N < -6000) ){ //27_08_20
                 Otkaz_Vkl = 1; Out_X4_6 = 1; // 27_03
                 Fault_20 = 1; Faultec = 1; } //printf("ток в TA2 \n");
           
               //if ((Opto_IN2 == 0) && (FAULT == 0) && (BV_Otkl == 0)) { // 04_05_22 СШ просил в вайбере &&(F2_KQF1 == 1) 
                //  Ch_IGCT = 20; IGCT_Test = 1;} //вкл. IGCT на 5мсек 17_06_21 - 5мсек поменял на 10мсек
               //else  
                 FAULT |= Opto_IN2; //04_07_22
                 if(((FAULT&0x01)&&(Fault_20 || (FAULT&0x02) || (V1_N < 860)))) Out_X4_6 = 1; //||Fault_26 //||(Fault_20&&Fault_26)
               //} //14_07_20
           #else
               if (((int16_t)I2_N_64ms > 50 )||((int16_t)I2_N_64ms < (-50))) { // if ( (SumI2_N > 6000)||(SumI2_N < -6000) ){ //27_08_20
                Otkaz_Vkl = 1; Out_X4_6 = 1; // 27_03
                Fault_20 = 1; Faultec=1;}
               if (  inp1_5 || inp1_6 ) {    //02_04_24
                Fault_15 = 1; Faultec=1;}   //printf("сбой сигналов блок-контактов \n"); //через 115мсек после F8_KM1 = 2;
               if (inp1_5 && inp1_6) {Out_X4_6 = 1; Otkaz_Vkl = 1;}// 17_12_19 надо ли?
           #endif	
              if ( inp1_4 ) { Fault_09=1; Faultec=1; } //
              II_Otkl = I2_N_64ms; //SumI2_N=0; 27_08_20
              Faultec = 1;
                break; //TA2_Control( );добавить контроль ТА2? //F8_KM1 = 1;
       
      case 8: 
           #ifdef AFB_40
              if ((Opto_IN2 == 0) && (FAULT == 0) && (BV_Otkl == 0)) { // 04_05_22 СШ просил в вайбере &&(F2_KQF1 == 1) 
                 Ch_IGCT = 20; IGCT_Test = 1; } //вкл. IGCT на 10мсек 17_06_21 - 5мсек поменял на 10мсек
              else { FAULT |= Opto_IN2; 
                 if(((FAULT&0x01)&&(Fault_20 || (FAULT&0x02) || (V1_N < 860)))) Out_X4_6 = 1; //||Fault_26 //||(Fault_20&&Fault_26)
               }
           #endif
        
        break;
      
      case 9: //F_TA2_Con=1; SumI2_N=0;                             //515/1300мсек   - 1800 надо ?? - уменьшил время с 1500 на 65мкс
              //Zapis_EEPROM_2( ); //14_03_20
              if (( (int16_t)I1_N_64ms > I_Gashen_max )||((int16_t)I1_N_64ms < (-I_Gashen_max)) )  //|| inp1_5 || inp1_6
              { Otkaz_Vkl = 1; Fault_23 = 1; Faultec = 1; Out_X4_6 = 1; }  
              I4_Otkl = I1_N; 
              if ( inp1_4 ) { Fault_09 = 1; Faultec = 1;}
              
              if (( F2_KQF1==2 )&&((FAULT&0x7DB7F7DF)==0)) { //if((Fault_15==1)&&((Out_X4_6|Fault_20|Fault_23)==0))F8_QS1 = 2; //TEMP //06_05_21
                F2_KQF1 = 0; 
                FAULT_O = FAULT; Zapis_EEPROM_2( ); //21_04_22
                F8_KM1 = 1; // 08_06_21 //23_12_21 temp
                f_Zapusk |= f_Low_24V; //printf("процесс откл-я без QS1 завершен \n"); //при отключение питания F8_KM1 = 1;
              #ifdef AFB_40 //1300мс
                C12_4 |= f_Low_24V; //31_03_21
              #endif
                }
                break; //printf("сбой отключения \n"); F2_KQF1 =2; процесс отключения завершен
      
      case 10: //F_TA2_Con=0;   //945 - 1900 надо  //870
              
              //SumI2_N=0; 
               //28_05_20 - все работало, но со слов СШ "добавил проверку блок-контактов БВ перед выкл. разъединителя.
           #ifdef AFB_40
              if (((int16_t)I2_N_64ms > 80 )||((int16_t)I2_N_64ms < (-80)))  // || inp1_5   temp-|| inp1_6  //27_08_20
              { Otkaz_Vkl = 1; Out_X4_6 = 1; Fault_20 = 1; Faultec = 1;} 
              if ( inp1_4 ) { Fault_09=1; Faultec=1; }// 27_03
              if (( (int16_t)I1_N_64ms > I_Gashen_max )||((int16_t)I1_N_64ms < (-I_Gashen_max)) )  
              { Otkaz_Vkl = 1; Fault_23 = 1; Faultec=1; Out_X4_6 = 1; }  
              
               if((((Fault_20 || Fault_23)||(inp1_5 && inp1_6))==0)&&((inp1_3 == 1)||(inp1_4 == 0))) //&&(Opto_IN2==0)|| Opto_IN2 // 28_04_21
                 F8_QS1 = 2; // ????  27_03
               else 
               {
                 Out_X4_6 |= ((((Fault_20 || Fault_23)!=0)||((inp1_3 == 0)&&(inp1_4 == 1)))!=0); //||(Opto_IN2==1)
                 FAULT |= (Opto_IN2); }// ?27_08_20 Fault_01
           #else
               if (((int16_t)I2_N_64ms > 50 )||((int16_t)I2_N_64ms < (-50)))  // || inp1_5   temp-|| inp1_6  //27_08_20
              { Otkaz_Vkl = 1; Out_X4_6 = 1; Fault_20 = 1; Faultec = 1;} 
              if ( inp1_4 ) { Fault_09=1; Faultec=1; }// 27_03
               //28_05_20 -
               if((((Fault_20 || Fault_23 )||(inp1_5 && inp1_6))==0) // 28_04_21 || Out_X4_6 - СШ попросил убрать
                  &&((inp1_3 == 1)||(inp1_4 == 0))){
                   if(((int16_t)I1_N_64ms < I_Gashen_max )&&((int16_t)I1_N_64ms > (-I_Gashen_max))) F8_QS1 = 2;   // ????  27_03// 05_06 добавил АутХ4_6 ==1
                   else { Otkaz_Vkl = 1; Fault_23 = 1; Faultec = 1; Out_X4_6 = 1; }}
               //else FAULT |= 1;
           #endif	
               break;
      case 11: F2_KQF1 = 0;  //5045   //26_01_20 перенес в алг-м QS1
               if((inp1_6 == 0)&&(inp4_5)&&(FAULT == 0))F8_KM1 = 1; 
               FAULT_O = FAULT; 
               Zapis_EEPROM_2( ); //21_04_22
               //Trig_Prot |= 0x8000; //флаг - запрет заряда //24_01_20
               if( (( inp1_3) ==1)&&((inp1_2 | inp1_4 )==0)){ 
                 uint8_t temp3 = ((FAULT & 0x02000000)==(uint32_t)0x00000000);
                 if((temp3)&&((f_Zapr_Zar)!=0)){
                   #ifdef AFB_40
                   C12_4 = 1;
                   #else
                   Vkl_VS1_2 = 1; F8_KM1 = 1; F8_KM2 = 1; //???
                   #endif
                 } //11_02 возм-н разряд через IGCT
               }//inp1_1 & по письму С.Ш. 17_01_20 (FAULT & (~(0x02000000))); &&(FAULT & 0xfdffffff)==0) 
               break; // printf("процесс откл-я с QS1 завершен \n");
      case 12: F2_KQF1=1; break; //if (V1_N < -1200) Formirovanie_Impulsov_VS1(5);  if (V1_N > 1200) Formirovanie_Impulsov_VS1(5);
      }
    }
    
    //if (F_TA2_Con==1) SumI2_N += I2_N; //TA2 сумма токов I*128 (int32_t)
     
    if ( T_Otkl > T_Faza_O [FAZA_Otkl] )
    {
      FAZA_Otkl++;
      F5_Faza = 1;
      F8_Proverka=0;
    }   
  T_Otkl++; //
  }
  else T_Otkl=0; 
  

  // процесс включения //// процесс включения //// процесс включения //
  if (F1_KQF1 != 0)
  {
    if( F6_Faza == 1 )// первый проход, смена фазы  200,20000,10000,10100,12100,12300,12500
    {
      F6_Faza = 0;  
      switch (FAZA_Vkl)
      {
      case 0: break; //резерв  
      case 1:  F8_Proverka=1; break; //0мсек printf("Фаза1 \n");запуск проверки перед вкл. QF1 
      
      case 2:  if((F0_Proverka==1) && (Fault_28==0)) {//10мсек printf("Фаза2 \n");разъед-ль вкл.если проверка удачно // 0,1сек
                  if((I1_N_64ms < 120)&&(I1_N_64ms > -120)) {
                  //F8_QS1=1; 25_03_21 0x00001000;
                 #ifdef AFB_40  
                   if(Opto_IN2==0)F8_QS1=1; else FAULT |= 0x00001; //Fault_13 //24_12_20
                 #else
                   F8_QS1=1;
                 #endif   
                  }
                  else FAULT |= 0x00010000; //Fault_17
               T_Vkl=20;
               //F_Cap_Contr = 1; //выставляет02ош
               }
               else { 
                #ifndef AFB_40
	         if(inp1_6==0){Fault_15=1; Faultec=1;} //inp1_6 - контактор ИКЗ
                #else
                 //Fault_13=1; FAULT |= 0x00001000; //26_03_21
                #endif 
                 Fault_12 = ((F_Cap_11 || F_Cap_12)==0);
	         F1_KQF1 = 0;  F8_QS1 = 2; //Otkaz_Vkl = 1; 05_01_21 по письму 29_12_20
	       } break; //Fault_12=1; Faultec=1; printf("Проверка не пройдена \n");
               
      #ifdef AFB_40
      case 3:  if((((inp1_2)== 0) || Opto_IN2 || Opto_IN3) || ((F_Cap_11 || F_Cap_12)==0) 
                  || (I2_N > 80) || (I2_N < (-80))) //printf("Фаза3 \n"); //5сек //по письму СШ 22_07_20 -||(F_Cap_11==0)||(F_Cap_12==0)
              {  
                F8_QS1 = 2; 
                if(I2_N > 0)F8_KM1=2; //13_04_21
                else F8_KM2=2;
                Fault_26 = Opto_IN3; // 26-или другая //10_06_21
                Fault_12 = ((F_Cap_11 || F_Cap_12)==0); // зарядка конд-в
                FAULT |= Opto_IN2; //18_05_21
                if((I2_N > 80) || (I2_N < (-80))) Fault_20 = 1;
                //Otkaz_Vkl = 1; //Fault_07=(inp1_2==0); //проверка сост-я разъед-ля, вкл-н?
                if(((FAULT&0x01)&&(Fault_20||Fault_26))||(Fault_20&&Fault_26)) Out_X4_6 = 1;
                Faultec=1; F1_KQF1 = 0; 
              }  
              else 
              {  
                F8_KM2=1; //21_05_20  KM2 - не включаем в 4-ке //05_03_21 - при ИКЗ включаем //if(F1_KQF1 == 2)-12_04_21
                F8_KM1=1;  // KM1 - включен, команда на всякий случай
              }
              break; //ИКЗ F8_KM1=1;?//для Вкл с испыт.
              
       case 4:  if (inp1_5) //printf("Фаза4 \n"); ??? inp1_5 || inp1_6 //5,05сек
              {
                 if((F_Cap_11==0)||(F_Cap_12==0)) { //((F_Cap_11 || F_Cap_12)==0)
                   Fault_12 = 1; Faultec=1; //F1_KQF1=0; 
                   if(I2_N > 0)F8_KM1=2; //13_04_21
                   else F8_KM2=2;
                   //F8_KM1=2; F8_KM2=2; //F8_QS1=2;-17_03_21 
                   if(Fault_20==0)f_QS_f12 = 1;  break; } //30_03_12 - if(Fault_20==0)
                 //Fault_26 |= Opto_IN3; //10_06_21
                 Faultec = 1; // 26-или другая
                 if(F1_KQF1==1) {T_Vkl = T_Faza_V [4] -40; //если без проверки пропускаем фазу //13_04_21
                 //#ifdef AFB_40     
                   //F8_KM2=2; //12_04_21
                 //#endif
                 }  
                 else F8_Pr_IKZ=(V3_N > 0) ? 1 : 2 ; //{ T_Faza_V [FAZA_Vkl]+=2040; } //запуск IKZ если включение с проверкой
              } //для Вкл прямо.
	      else 
              {
                F8_QS1=2; Fault_15=1; Faultec=1;   
                if(I2_N > 0)F8_KM1=2;
                else F8_KM2=2;
              }
              break;    //if (F1_KQF1==2)F8_QF1=1;//QF1 вкл-ть
       
       //case 5: break; //промжуточная фаза по СШ //5,25сек
       
       case 5: F8_KM1 = 2; F8_KM2 = 2; if(Fault_15 || Fault_12) F1_KQF1 = 0; //13_04_21
         
              if (F1_KQF1==2) //printf("Фаза5 \n"); //6,05сек 
              { 
                if(F0_Pr_IKZ &&((FAULT || Opto_IN2)==0)){ F8_QF1=1; }  //для Вкл с испыт. //  //14_07_20
                else {
                  //FAULT |= (Opto_IN2 << 4); //СШ не нравится 25_03_21
                  Otkaz_IKZ = 1; Faultec=1; // Fault_28=1; printf("Отказ по ИКЗ \n");+LED, отказ вкл. по ИКЗ
                  Status_Flag |=0x0040; 
                  if((I2_N > 50) || (I2_N < (-50)) || (inp1_5 && inp1_6)){ //
                    Otkaz_Vkl = 1; Out_X4_6 = 1; F1_KQF1 = 0;
                    I2_Ntemp = I2_N; TEMP_KM = (inp1_5 << 1) | inp1_6;
                    if((I2_N > 50) || (I2_N < (-50))) Fault_20 = 1;  //26_01_20 Коля сказал что F8_QS1=2; - делать можно, а потом отказался подтвердить 03_06_20
                    F8_KM1=2; F8_KM2=2;  //03_06_20 а СШ сказал выдавать УРОВ вместо откл QS1
                  } 
                  else { 
                    F8_QS1 = 2; F1_KQF1 = 0; 
                  }
                } 
              }
              else { 
                if((FAULT || Opto_IN2)==0 )F8_QF1=1;
                else {FAULT |= (Opto_IN2 << 4); }//F1_KQF1 = 0;14_07_20
              }  break; 
              
      #else
       case 3:  if((inp1_2==0)) //printf("Фаза3 \n"); //5сек //по письму СШ 22_07_20 -||(F_Cap_11==0)||(F_Cap_12==0)
              {  
                F8_QS1 = 2; F8_KM1=2;
                 //Fault_07=(inp1_2==0); //Otkaz_Vkl = 1; 05_01_21 по письму 29_12_20
                F1_KQF1 = 0; } //break; //проверка сост-я разъед-ля, вкл-н? зарядка конд-в  //по письму СШ 22_07_20 -Fault_12 = (F_Cap_11==0)||(F_Cap_12==0); Faultec=1;
              else 
              {  
                F_Cap_Contr = 1; 
                F8_KM2=1; 
                F8_KM1=1; }// KM1 - включен, команда на всякий случай
              break; //ИКЗ F8_KM1=1;?//для Вкл с испыт.
                    
      
      case 4:  if (inp1_5 && inp1_6) //printf("Фаза4 \n"); ??? inp1_5 || inp1_6 //5,05сек
              {
                 if(FAULT & 0x06) { //(F_Cap_11==0)||(F_Cap_12==0)
                   Fault_12 = 1; Faultec=1; F1_KQF1=0; F8_KM1=2; F8_KM2=2; f_QS_f12 = 1; break; } //F8_QS1 = 2;-17_03_21
                 if(F1_KQF1==1) {T_Vkl = T_Faza_V [4] +1; 
                 #ifdef AFB_40     
                 F8_KM2=2;
                 #endif
                 }  //если без проверки пропускаем фазу //
                 else F8_Pr_IKZ=(V3_N > 0) ? 1 : 2 ; //{ T_Faza_V [FAZA_Vkl]+=2040; } //запуск IKZ если включение с проверкой
              } //для Вкл прямо.
	      else 
              {
                 F8_QS1 = 2; Fault_15 = 1; Faultec = 1; 
                 F1_KQF1 = 0; F8_KM2 = 2; F8_KM1 = 2;
              }
              break;    //if (F1_KQF1==2)F8_QF1=1;//QF1 вкл-ть
              
       //case 5: break;//промжуточная фаза по СШ
             
      case 5: if (F1_KQF1==2) //printf("Фаза5 \n"); //6,05сек
              { 
                if(F0_Pr_IKZ &&((FAULT )==0)){ F8_QF1=1; }  //|| Opto_IN2для Вкл с испыт. //  //14_07_20
                else {
                  //FAULT |= (Opto_IN2 << 4); 
                  //Otkaz_IKZ = (F0_Pr_IKZ==0); Faultec=1; // Fault_28=1; printf("Отказ по ИКЗ \n");+LED, отказ вкл. по ИКЗ
                  Status_Flag |=0x0040; 
                  if((I2_N > 50) || (I2_N < (-50)) || (inp1_5 && inp1_6 && Otkaz_IKZ)){  //
                    Otkaz_Vkl = 1; 
                    Out_X4_6 = 1; 
                    F1_KQF1 = 0;
                    I2_Ntemp = I2_N; TEMP_KM = (inp1_5 << 1) | inp1_6;
                    if((I2_N > 50) || (I2_N < (-50)))Fault_20=1;  //26_01_20 Коля сказал что F8_QS1=2; - делать можно, а потом отказался подтвердить 03_06_20
                    //else Fault_15=1;
                    F8_KM1=2; F8_KM2=2;  //03_06_20 а СШ сказал выдавать УРОВ вместо откл QS1
                  } 
                  else { 
                     F1_KQF1 = 0; F8_KM1=2; F8_KM2=2; if((FAULT&0x00010000)==0) F8_QS1=2;  //03_11_20 //29_04_21
                  }
                } 
              }
              else { 
                if( (FAULT)==0 )F8_QF1=1; //|| Opto_IN2
                //else {FAULT |= (Opto_IN2 << 4); }//F1_KQF1 = 0;14_07_20
              }  break;        
              
       #endif
               
              
      case 6:  if((inp1_4==0)||(inp1_3==1)){ //Out_X4_6=1; F8_QF1=2; printf("Фаза6 \n"); //5,15сек
        Fault_09=1; Faultec=1; //F_Cap_Contr = 0; 
        if(V1_N < 100){Out_X4_6=1; FAULT|=0x2;}
        if(V2_N < 100){ FAULT|=0x4;}
      } 
        else {F1_KQF1 = 0;  } break; //printf("Процесс вкл-я выполнен \n");контроль сост-я контактов БВ
      
      case 7:     F1_KQF1 = 0; break;//Otkaz_Vkl = 1; Out_X4_6 = 1;-28_04_21   F8_QS1 = 2; -14_07_20  printf("Фаза7 \n");if( Fault_09==1 || F2_QF1==0 ) {}  // 5,25сек
              //else F1_KQF1 = 0;  break; //TA2_Control( );
      //case 7:   // F2_KQF1 =2; процесс отключения завершен
      }
    }
    
    if ( T_Vkl > T_Faza_V [FAZA_Vkl] ) //абсолютные знач-я /или длительность фазы
    {
      FAZA_Vkl++;
      F6_Faza = 1;
      //T_Vkl=0; //длительность фазы
    }   
  
    T_Vkl++;
    
  }
  else T_Vkl=0;
     
}

////////проверка  ИКЗ ////////////проверка  ИКЗ ////////////проверка  ИКЗ ////////////проверка  ИКЗ //// сколько Х-?
uint8_t F8_Pr_IKZ=0;
uint8_t F0_Pr_IKZ=0;
uint16_t N_ltd=0;
extern uint8_t Ch_KM2_o;


void IKZ_Control (void) //10 ms 
{
  //F0_Pr_IKZ=1;//TEMP //TEMP //TEMP //TEMP //TEMP //TEMP 
  static uint8_t n; //
  static int32_t S; //сумма токов 

   //новый алгоритм ИКЗ 
  if(F8_Pr_IKZ!=0)
  { 
    #ifdef AFB_40
    if (I2_N > 0) S+= (int32_t)((V3_N*4)/I2_N); // 
    else S += 5000;
    #else
    if (I2_N > 0) S+= (int32_t)((V3_N*4)/I2_N); // 
    else S += 5000;
    #endif
    
    n++;
    if ((n > 30)){ //30
       V3_TEMP = V3_N;
       if(S > (int32_t)3300) S = 3300;
       //else if(S < 0) S = 0;
       LTD_Izmer = S*10; //5_11_20
       usRegHoldingBuf[55]=LTD_Izmer;
       if(S > LTD_Threshold) {
         
    #ifdef AFB_40 
         F8_KM2=2; //?
    #endif
         
         F0_Pr_IKZ=1;  
       }// 
       else { 
         F0_Pr_IKZ=0; //F8_KM1=2; F8_KM2=2;
         if (I2_N > 0){F8_KM1=2; Ch_KM2_o=74;}
         else {F8_KM2=2; } //07_05_21
         //Otkaz_IKZ = (F0_Pr_IKZ==0); Faultec=1; 
       }
       F8_Pr_IKZ=0; n=0; S=0;
    }
  }
  
  V1_N_ = V1_N;
  V2_N_ = V2_N;
  
}

//F8_IGCT=0;
#ifdef AFB_40
  extern uint8_t F_Cap_Contr;
  uint8_t IGCT_Test = 2;

  void IGCT_Control ( void ) //500mks
  {
    //if(F8_IGCT==1 && F1_IGCT==0){ F1_IGCT = 1; Ch_IGCT = T_Vkl_IGCT << 1; }
    if(Ch_IGCT > 0) {
      Ch_IGCT --; F_Cap_Contr =0;
      IGCT_On; //GPIOE->BSRR = GPIO_BSRR_BR1
    }
    else {
      IGCT_Off; //27_02
      if((IGCT_Test ==1)&&(V1_N>(int16_t)110)&&((FAULT & 0x08)==0)) { // 9_04_21
        FAULT |= 0x08000000; Fault_28=1; f_QS_f12 = 1;} //fault_28
      if(IGCT_Test ==1)IGCT_Test=0; 
    }
  }
#endif