// Электронный редуктор-Сервоконтроллер
// Контроллер STM32F103С8T6
#include "stm32f10x.h"

#define eeprom 0x0801fc00                        // Адрес памяти типа EEPROMа
#define str 30                                   // Переменная длинны массива
#define dtread 60                                // Переменная длинны массива 
#define Err 0xffffffff                           // Число ошибки в операции захвата числа
#define freq 88000000                            // Частота проца в разогнанном состоянии
#define limitRUN 20                              // Лимит на занятость
#define limitWARN 150                            // Лимит предупреждение
#define limitPgain 30000                         // Лимит пропорц. константы
#define limitIgain 30000                         // Лимит интегральной. константы
#define limitDgain 30000                         // Лимит диференциальной. константы
#define limitVgain 30000                         // Лимит скоростной. константы
#define limitErrorH 30000                        // Лимит ошибки макс
#define limitErrorL 70	         	             // Лимит ошибки минимум
#define limitServoTimeH 30000                    // Лимит серво времени макс
#define limitServoTimeL 1500                     // Лимит серво времени минимум
#define limitPWMfreqH 2000                       // Лимит делителя частоты ШИМ макс
#define limitPWMfreqL 5                          // Лимит делителя частоты ШИМ макс
#define limitPWM1 450                            // Лимит делителя сектор 1 
#define limitPWM2 400                            // Лимит делителя сектор 2
#define limitPWMmaxH 512                         // Лимит ШИМ макс
#define limitPWMmaxL 10                          // Лимит ШИМ макс
#define limitMuldiv 100000                       // Лимит множителя
#define limitdeadtime 10000                      // Лимит программного МертвогоВренени
#define limitCurrent 1000                        // Лимит тока
#define deadtimebase 100                         // Базовое мертвое время
#define plot_freq 1000                           // Значение задержки графика
#define savecode 111                             // Код записи во флеш
#define acp1_con 8                               // Количество усреднений АЦП
#define acp1_null 3215                           // Уровень 0 датчика тока
#define sumH    400                              // Лимит интегральной ошибки
#define sumL    -400                             // Лимит интегральной ошибки
#define velH    400                              // Лимит ошибки скорости
#define velL    -400                             // Лимит ошибки скорости
#define limitTermo1 0x00000010                   // степы вкл.измерения температуры
#define limitTermo2 0x00020000                   // степы чтения температуры и переключение датчика

extern void Reset_Handler();

// PA0, PA1 - ШИМ1 двигатель№1 с энкодером двигательным 
// PA2, PA3 - ШИМ2 двигатель№2 с энкодером двигательным

// PA8, PA9 - энкодер №1 на таймер №1 задающий          || если подключен энкодер
// PA8 - Step, PA9 - Dir			                    || если подключен управлятор

// PB4, PB5 - энкодер №1 на таймер №3 двигательный№1    ||remap
// PB6, PB7 - энкодер №2 на таймер №4 двигательный№2

// PB12..PB15 - экран D4..D7
// PB0 - экран E
// PB1 - экран RS

// PA10, PA15 - сигналы N1..N4 тут клавиши или что угодно
// PA6, PA7 - аналоговые сигналы A0,A1
// PA4 - датчик температуры Т1 1-wire отк.коллектор
// PA5 - выход реле обдува  T2 отк.коллектор

// PC13 - диод на плате сигнал ENABLE
// PB9 - стоит в сервоцикле, сигнал PWM
// PB8 - SRUN серво ЗАНЯТ BUSY
// PB3 - SERROR ошибка

// USART3 PB10 на 11max232, PB11 на 12 max232

long pgain;		                                //Proportional gain 
long igain;				                        //Integral gain
long dgain;				                        //Derivative gain
long vgain;				                        //Velocity gain
long PSK_gain;                                  //делитель ШИМ
long Servo_time;                          	    //время сервоцикла
long limitERROR;                        	    //ОШИБКА
long current_max;                               //максимальный ток
float muldiv;                                   //переменная умножения
uint8_t encoderM;                               //переменная счета энкодера 1-1фрА 2-1фрВ 2-2фр
uint32_t deadtime;                              //мертвое время
int16_t PWMmax;                                 //переменная максимума ШИМ
int16_t PWM;
int16_t cur_err;                                  //переменная ошибки по току

uint8_t FLAG;                                     //1-пришла команда
                                                  //2-трансляция графика ошибки
                                                  //4-Тест
                                                  //8-Вкл зависимости частоты ШИМ от силы нажатия
                                                  //16-Включение токового теста им.Куркова
                                                  //32-трансляция графика усилия
                                                  //64-трансляция графика тока
                                                  //128-ОШИБКА

uint8_t FLAG2;                                    //1-трансляция графика тока/10                                                

char string[str];
char dataread[dtread];
uint8_t DS18data[9];
uint8_t  encoder_count1;
uint16_t encoder_count2;
uint16_t encoder_mul;
uint16_t last_error_SD;
uint16_t last_error_SD2;
uint16_t last_error_SD3;                           
uint16_t test_zapoln;                             //переменные теста им.Куркова
uint16_t test_time_open;
uint16_t test_count;
uint16_t test_time_count;
long systick;
uint16_t    acp1_count;                           //Cчетчик АЦП
uint32_t    acp1_data;                            // буфер АЦП
uint16_t    current;                              // ток
uint16_t last_encoderA;                           //Прошлая позиция А энкодера
long positionA;                                   //Позиция А
uint16_t last_encoderB;                           //Прошлая позиция B энкодера
long positionB;                                   //Позиция B
float counter_step;             		          //Количество шагов которые необходимо выполнить

float last_error;       
float last_result;

float period;                                     //переменные для предрасчета
float periodrecip;
float pgainF;
float igainF;
float dgainF;
float vgainF;
uint8_t numID;                                    //количество датчиков
uint8_t numID_count=0;                            //счётчик последовательности измерений
uint32_t termo_count=0x00000000;                  //счётчик измерений
uint8_t t_on;                                     // температура включения куллера
uint8_t t_off;                                    // температура выключения куллера
uint8_t t_error1;                                 // температура ошибки1
uint8_t t_error2;                                 // температура ошибки2

uint8_t readcount=0;                  		      //переменая адреса чтения USART
uint8_t last_readcount;
uint16_t usart_graf_count;              	      //переменная графика

void SystemInit(){}

void pause(unsigned long tick){  
  while(tick!=0){tick--;}}

void strobe(void){
GPIOB->BSRR|=(1<<0);			//Set E
pause (1200);
GPIOB->BSRR|=(1<<16);			//Res E
GPIOB->BSRR|=0XF0000000;}                                   // Res PB12..14

//+++++  Чтение из флеш памяти  +++++
uint32_t flash_read(uint32_t address) {
  return (*(__IO uint32_t*) address);}

//+++++ Разблокировка флэш +++++
#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
void flash_unlock(void) {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;}

//+++++ Блокировка флэш +++++
void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;}

//Функция возврщает true когда можно стирать или писать память.
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);}

//Функция стирает одну страницу. В качестве адреса можно использовать любой
//принадлежащий диапазону адресов той странице которую нужно очистить.
void flash_erase_page(uint32_t address) {
  while(!flash_ready());
  FLASH->CR|= FLASH_CR_PER; //Устанавливаем бит стирания одной страницы
  FLASH->AR = address; // Задаем её адрес
  FLASH->CR|= FLASH_CR_STRT; // Запускаем стирание 
  while(!flash_ready())
    ;  //Ждем пока страница сотрется. 
  FLASH->CR&= ~FLASH_CR_PER; //Сбрасываем бит обратно
}

//+++++ запись во Флэш +++++
void flash_write(uint32_t address,uint32_t data) {
  while(!flash_ready()) //Ожидаем готовности флеша к записи
    ;
  *(__IO uint16_t*)address = (uint16_t)data; //Пишем младшие 2 бата
  while(!flash_ready())
    ;
  address+=2;
  data>>=16;
  *(__IO uint16_t*)address = (uint16_t)data; //Пишем старшие 2 байта
  while(!flash_ready())
    ;
}

//+++++ Читаем переменные +++++
void LOAD(void){
  	uint32_t data;
	pgain=flash_read(eeprom);
	igain=flash_read(eeprom+4);
	dgain=flash_read(eeprom+8);
	PSK_gain=flash_read(eeprom+12);
	Servo_time=flash_read(eeprom+16);
	limitERROR=flash_read(eeprom+20);
	data=flash_read(eeprom+24);
	muldiv=*(float*)&data;
	encoderM=flash_read(eeprom+28);
	deadtime=flash_read(eeprom+32);
	data=flash_read(eeprom+36);
    if (data==1) FLAG|=8;
	PWMmax=flash_read(eeprom+40);
    vgain=flash_read(eeprom+44);
    current_max=flash_read(eeprom+48);
    t_on=flash_read(eeprom+52);
    t_off=flash_read(eeprom+56);
    t_error1=flash_read(eeprom+60);
    t_error2=flash_read(eeprom+64);

	if(pgain<0||pgain>limitPgain)pgain=0; 
	if(igain<0||igain>limitIgain)igain=0;	
	if(dgain<0||dgain>limitDgain)dgain=0;	
	if(PSK_gain<limitPWMfreqL||PSK_gain>limitPWMfreqH)PSK_gain=limitPWMfreqL;	
	if(Servo_time<limitServoTimeL||Servo_time>limitServoTimeH)Servo_time=limitServoTimeL;	
	if(limitERROR<limitErrorL||limitERROR>limitErrorH)limitERROR=limitErrorL;
	if(muldiv<(1/limitMuldiv)||muldiv>limitMuldiv)muldiv=1;
	if(encoderM<1||encoderM>3)encoderM=1;
	if(deadtime<1||deadtime>limitdeadtime)deadtime=deadtimebase;
	if(PWMmax<limitPWMmaxL||PSK_gain>limitPWMmaxH)PSK_gain=limitPWMmaxL;
	if(vgain<0||vgain>limitVgain)dgain=0;
	if(current_max<0||current_max>limitCurrent)current_max=0;    
	
}

//+++++ Сохраняем переменные +++++
void SAVE(void){
          RCC->APB1ENR&=~RCC_APB1ENR_WWDGEN;
	__disable_irq();
	RCC->CR|=RCC_CR_HSION;	
	RCC->CFGR|=RCC_CFGR_SW_HSE;	
	RCC->CFGR&=~RCC_CFGR_SW_PLL;
	while ((RCC->CR & RCC_CFGR_SWS)==RCC_CFGR_SWS_HSE) {} // Ожидание готовности	
          flash_unlock();
          flash_erase_page(eeprom);
	FLASH->CR |= FLASH_CR_PG; 		//Разрешаем программирование флеша
	
	flash_write(eeprom,pgain);
	flash_write(eeprom+4,igain);
	flash_write(eeprom+8,dgain);
	flash_write(eeprom+12,PSK_gain);
	flash_write(eeprom+16,Servo_time);
	flash_write(eeprom+20,limitERROR);	
	flash_write(eeprom+24,*(uint32_t *)&muldiv);
	flash_write(eeprom+28,encoderM);
	flash_write(eeprom+32,deadtime);
    if ((FLAG&8)==8) flash_write(eeprom+36,1);
	flash_write(eeprom+40,PWMmax);
	flash_write(eeprom+44,vgain);
    flash_write(eeprom+48,current_max);
    
    flash_write(eeprom+52,t_on);
    flash_write(eeprom+56,t_off);
    flash_write(eeprom+60,t_error1);
    flash_write(eeprom+64,t_error2);
	

	FLASH->CR &= ~(FLASH_CR_PG); 		//Запрещаем программирование флеша	
	flash_lock();	
	RCC->CFGR|=RCC_CFGR_SW_PLL;
	RCC->CFGR&=~RCC_CFGR_SW_HSE;	
	__enable_irq();
	RCC->CR&=~RCC_CR_HSION;	
  	RCC->APB1ENR|=RCC_APB1ENR_WWDGEN;}

uint8_t VERIFY(void){
    	uint32_t data;
	float datafloat;
	if (pgain!=flash_read(eeprom)) return 1;
	if (igain!=flash_read(eeprom+4)) return 1;
	if (dgain!=flash_read(eeprom+8)) return 1;
	if (PSK_gain!=flash_read(eeprom+12)) return 1;
	if (Servo_time!=flash_read(eeprom+16)) return 1;
	if (limitERROR!=flash_read(eeprom+20)) return 1;
	data=flash_read(eeprom+24);
	datafloat=*(float*)&data;
	if (muldiv!=datafloat) return 1;
	if (encoderM!=flash_read(eeprom+28)) return 1;
	if (deadtime!=flash_read(eeprom+32)) return 1;
	if (PWMmax!=flash_read(eeprom+40)) return 1;
	if (vgain!=flash_read(eeprom+44)) return 1;
    if (current_max!=flash_read(eeprom+48)) return 1;
    
    if (t_on!=flash_read(eeprom+52)) return 1;
    if (t_off!=flash_read(eeprom+56)) return 1;
    if (t_error1!=flash_read(eeprom+60)) return 1;
    if (t_error2!=flash_read(eeprom+64)) return 1;   
    
    
    
return 0;}

//+++++ ПОДПРОГРАММА ЗАПИСИ В LCD +++++
void write(unsigned char byte, unsigned char com){
if (com==1) GPIOB->BSRR|=(1<<17);                           // res RS КОМАНДА 1
if (com==0) GPIOB->BSRR|=(1<<1);                            // set RS ДАННЫЕ  0
GPIOB->BSRR|=((byte&0xf0)<<8); 
strobe();
GPIOB->BSRR|=((byte&0x0f)<<12); 
strobe();}

//+++++ инициализация LCD +++++
void iniciale(void){
  	pause (10000);  
	GPIOB->BSRR|=(1<<17);                                   // res RS
	GPIOB->BSRR|=0XF0000000;	                            // Res PB12..14
    GPIOB->BSRR|=((0x30&0xf0)<<8);
	strobe();
	GPIOB->BSRR|=((0x30&0xf0)<<8); 
	strobe();
	GPIOB->BSRR|=((0x30&0xf0)<<8); 
	strobe();		
	GPIOB->BSRR|=((0x20&0xf0)<<8); 
	strobe();	
	write (0x01,1);			                                //Clear Display
	pause (1500);
	write (0x06,1);
	write (0x0c,1);}

//+++++ Установка курсора +++++
void SetCursor(unsigned char x, unsigned char y){
   uint8_t data=0x80;
   if (x==0) x=1;
   if (x>16) x=0x0f;
   x--;
   if (y==2) data|=0x40;
   if (y==3) data|=0x10;  
   if (y==4) data|=0x50;  
   data|=x;
   write(data,1);}

//+++++ Вывод текста на экран по координатам ++++++  
void print(unsigned char x, unsigned char y,char *string){
   SetCursor (x,y);
   for (uint8_t z=0; string[z]!=0; z++){ 
   write(string[z],0);}}

//+++++ CONVERT INT2STRING +++++
char *con_int2strD(long x, unsigned char a, unsigned char b){
	char q=0;
	if (x<0) {x=~x+1;q=1;}
  char antdes=0;           // a - количество разрядов
    if(x>999999999)        // в - на каком разряде запятая
        antdes=10; 
    else if(x>99999999)
        antdes=9;
    else if(x>9999999)
        antdes=8;
    else if(x>999999)
        antdes=7;
    else if(x>99999)
        antdes=6;
    else if(x>9999)
        antdes=5;
    else if(x>999)
        antdes=4;
    else if(x>99)
        antdes=3;
    else if(x>9)
        antdes=2;
    else 
        antdes=1;
    if (a>0) antdes=a;
    if (antdes<=b) antdes=b+1;
    if (q==1) antdes++;
	for(char i=0;i<antdes;i++)
          {string[antdes-i-1]=(x%10)+48;
          x=x/10;}

          if (b>0){
	  for (char i=antdes;(antdes-b)<=(i-1);i--)
	  {string[i]=string[i-1];}
	   string[antdes-b]=',';
	    antdes++;}

	string[antdes]=0;
	
	if (q==1) string[0]='-';
	
	return string;}

//+++++ Прерывание собаки +++++
void WWDG_IRQHandler(void){
	WWDG->SR&=~WWDG_SR_EWIF;	// Сброс флага
	WWDG->CR|=0X7F;}		    // Обновляем значение таймера

void BUSY(uint8_t a){
	if (a==1) GPIOB->BSRR|=(1<<8); 
	if (a==0) GPIOB->BSRR|=(1<<24);   
}

//+++++ Управление выходом Т2 выход на куллер
void COOLER(uint8_t a){
	if (a==1) GPIOA->BSRR|=(1<<5); 
	if (a==0) GPIOA->BSRR|=(1<<21);  
}

void SERROR(uint8_t a){
	if (a==1) GPIOB->BSRR|=(1<<3); 
	if (a==0) GPIOB->BSRR|=(1<<19);
}

void WARN(uint8_t a){
	if (a==1) GPIOC->BSRR|=(1<<13); 
	if (a==0) GPIOC->BSRR|=(1<<29);
}

void SysTick_Handler(void){  
    if ((FLAG&16)==16){
 	GPIOB->BSRR|=(1<<9);          //диод on  
    systick--;
	GPIOB->BSRR|=(1<<25);  
    }
    else{
 	GPIOB->BSRR|=(1<<9);          //диод on
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    uint16_t encoder=TIM3->CNT;
	uint16_t encoderb=TIM1->CNT;
	int16_t encoder2b=last_encoderB-encoderb;
	positionB+=encoder2b;	       
	last_encoderB=encoderb;
	counter_step+=(float)encoder2b*muldiv; 

	int16_t encoder2=last_encoderA-encoder;
	last_encoderA=encoder;
	positionA+=encoder2;	          // где мы
	counter_step-=encoder2;           // сколько шагов нужно сделать
    
    float vel=(counter_step-last_error)*vgainF;   
    if (vel>velH) vel=velH;
    if (vel<velL) vel=velL;
    float sum = last_result + igainF * period * counter_step;
    if (sum>sumH) sum=sumH;
    if (sum<sumL) sum=sumL;
	float diff = dgainF * periodrecip * (counter_step - last_error);
	float result = pgainF * counter_step + sum + diff+vel;

	last_result = sum;
	last_error=counter_step;
	
	if (result>=0) {
      encoder=(short)result;
	  if (encoder>PWM) encoder=PWM; 

	  if (TIM2->CCR1!=0){
	  TIM2->CCR1=0;	    
	  pause (deadtime);}	    
	  TIM2->CCR1=0;
	  TIM2->CCR2=encoder;}
	
	if (result<=0) {
	  encoder=~(short)result+1;      
	  if (encoder>PWM) encoder=PWM;

	  if (TIM2->CCR2!=0){
      TIM2->CCR2=0;	    
	  pause (deadtime);}
      TIM2->CCR2=0;
	  TIM2->CCR1=encoder;}          
	GPIOB->BSRR|=(1<<25);}          //диод off   
}

uint8_t encoder(void){
  	encoder_count1++;
	if (encoder_count1==0) {
	  if (encoder_count2==0) encoder_mul=1; 
	  if (encoder_count2==1) encoder_mul=1; 
	  if (encoder_count2==2) encoder_mul=10;
	  if (encoder_count2==3) encoder_mul=100;
	  if (encoder_count2==4) encoder_mul=1000;	  
	  encoder_count2=0;}
	
	if (TIM4->CNT>0x8001) {encoder_count2++;TIM4->CNT=0x8000;return 1;}  
	if (TIM4->CNT<0x7fff) {encoder_count2++;TIM4->CNT=0x8000;return 2;}
	if ((GPIOB->IDR & GPIO_IDR_IDR5)==0){
	    uint16_t DATA=0;
	    while ((GPIOB->IDR & GPIO_IDR_IDR5)==0){
		DATA++;
		if (DATA>10000){ 
		  while ((GPIOB->IDR & GPIO_IDR_IDR5)==0){}  
		  return 3;}}}
	
	    return 0;}

void CLS(){
    print (1,1,"                ");
	print (1,2,"                ");}

//+++++ ПЕРЕДАЧА ПО USART +++++
void USART_Send(char chr) {
  while(!(USART3->SR & USART_SR_TC));
  USART3->DR = chr;}

//+++++ ПОСЫЛКА КОМАНДЫ ПО USART +++++
void USART_COMM(char *string){
  for (char a=0; a<str; a++){
       if (string[a]==0) break;
       USART_Send(string[a]);}  
}

//+++++ Прерывание USART +++++
void USART3_IRQHandler(void) {
	if (USART3->SR & USART_SR_RXNE) {
		dataread[readcount]=USART3->DR;
		if (dataread[readcount]==0x0d) FLAG|=1;
		if (readcount<dtread)readcount++;}}

//+++++ Прерывание АЦП +++++
void ADC1_2_IRQHandler(void){
  if ((ADC1->SR&ADC_SR_EOC)==ADC_SR_EOC){
    ADC1->SR&=~ADC_SR_EOC;                                  // Сбрасываем флаг EOC
    if (acp1_count!=0){
      acp1_count--;
      acp1_data+=ADC1->DR;}}
}    

//+++++ ГРАФИК +++++
void graf(long data){
  	if (data>126) data=126;
	if (data<-126) data=-126;
	data=~(data-128);
	data=(data+1>>1)|0x80;
	data&=0xff;
	USART_Send(data);}
  
void Hope_parameter(){
  	SysTick_Config(Servo_time);
	period=1.0/(freq/Servo_time);
	periodrecip = 1.0 / period;  
          pgainF=pgain*0.01;
          igainF=igain*0.01;
          dgainF=dgain*0.0001;
          vgainF=vgain*0.01;          
}

uint8_t get_mass_num(char a){
         char b=dataread[a];
         if (b<0x30) return 0xff;
         if (b>0x39) return 0xff;
         b=b-0x30;
         return b;}

// Множитель a*b
uint32_t pow(uint8_t a, uint8_t b){
	uint32_t c = 1;
	uint8_t i;
	for (i = 0; i < b; i++) {
		c *= a;}
	return c;}

uint16_t brr_calc(uint32_t data){
#define CR1_OVER8_Set             ((u16)0x8000)
#define USARTx                     USART3
	uint32_t integerdivider;
	uint32_t apbclock=freq/2;
  /* Determine the integer part */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0){
    /* Integer part computing in case Oversampling mode is 8 Samples */
     integerdivider = ((25 * apbclock) / (2 * (data)));}    
	else{ /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
    /* Integer part computing in case Oversampling mode is 16 Samples */
     integerdivider = ((25 * apbclock) / (4 * (data)));}    
  
  uint32_t tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  uint32_t fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0){  
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);} 
	  else{ /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);}
  
  /* Write to USART BRR */
  return (uint16_t)tmpreg;}

//+++++ Чтение из массива целого числа по указателю +++++
long read_data(uint8_t a){
         long data=0;    
         uint8_t count=0;    
         while(1){
         uint8_t num=get_mass_num(a);             //насчитываем целую часть
         if (num==0xff) break;
         uint32_t add_num=pow(10,count);
         count++;
         a--;
         for (uint8_t st=0; st<num; st++){
         data+=add_num;}}
         return data;}  

//+++++ Поиск ПРОБЕЛА в массиве +++++
uint8_t read_noop(uint8_t a){
        uint8_t b;
        while(1){
        b=dataread[a];
        a--;          
        if (b==' ') break;
        if (a<3) {a=0xff; break;}
        }
        return a;}

void get_param_temp(void){
        if (readcount==3) return;
        if (readcount>0x17) return;
        uint8_t a=readcount-2;
        t_error1=read_data(a);
        a=read_noop(a);
        if (a==0xff) return;        
        t_error2=read_data(a);       
        a=read_noop(a);
        if (a==0xff) return;  
        t_off=read_data(a);        
        a=read_noop(a);
        if (a==0xff) return;
        t_on=read_data(a);
        }

// +++++ Чтение параметров для теста им.Куркова 
void get_param_test(void){
        if (readcount==3) return;
        if (readcount>0x17) return;
        uint8_t a=readcount-2;
        test_time_count=read_data(a);
        a=read_noop(a);
        if (a==0xff) return;        
        test_count=read_data(a);       
        a=read_noop(a);
        if (a==0xff) return;  
        test_time_open=read_data(a);        
        a=read_noop(a);
        if (a==0xff) return;
        test_zapoln=read_data(a);
        }

long get_param(void){
  	long data=0;
	char a=readcount-2;
	uint8_t b;	
         if (readcount==2) return Err;	
         if (readcount>9) return Err;
         b=get_mass_num(a);
         if (b==0xff)return Err; 
         data=data+b;
         if (a==1) return data;
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+10*b;         
         if (a==1) return data;         
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+100*b;         
         if (a==1) return data;         
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+1000*b;         
         if (a==1) return data;         
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+10000*b;         
         if (a==1) return data;         
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+100000*b;         
         if (a==1) return data;
         a--;
         b=get_mass_num(a);
         if (b==0xff)return Err;         
         data=data+1000000*b;         
         if (a==1) return data;         
return Err;}

//+++++ MENU +++++
void menu(void){
	CLS();
	uint8_t num_menu=1; 
	uint8_t data;
  
	while(1){	
	if (num_menu==1){
	print (1,1,"P Gain      ");	
	print (2,2,con_int2strD((pgain),5,0));}	
	  
	if (num_menu==2){
	print (1,1,"I Gain      ");	
	print (2,2,con_int2strD((igain),5,0));}	
	
	if (num_menu==3){
	print (1,1,"D Gain      ");	
	print (2,2,con_int2strD((dgain),5,0));}	

          if (num_menu==4){
	print (1,1,"PSK PWM     ");	
	print (2,2,con_int2strD((PSK_gain),5,0));}
	
	if (num_menu==5){
	print (1,1,"SERVO TIME  ");
	print (2,2,con_int2strD((Servo_time),5,0));}	
	
	if (num_menu==6){
	print (1,1,"ERROR       ");
	print (2,2,con_int2strD((limitERROR),5,0));}	

	if (num_menu==7){
	print (1,1,"SAVE        ");
	print (1,2,"            ");}
	
	if (num_menu==8){
	print (1,1,"EXIT        ");
	print (1,2,"            ");}	
		
	data=encoder();
	if (data==1){
	num_menu++;
	if (num_menu>=9) num_menu--;}
	if (data==2){
	num_menu--;
	if (num_menu==0) num_menu=1;}	
	if (data==3){
		  if (num_menu==8) {
	    		break;}
		  
	            if (num_menu==1){		//Изменяем параметр №1
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) pgain=pgain+encoder_mul;
		  if (data==2) pgain=pgain-encoder_mul;
		  if (pgain>limitPgain) pgain=limitPgain;		 
		  if (pgain<0) pgain=0;
		  Hope_parameter(); 		  
		  print (2,2,con_int2strD((pgain),5,0));
		  if (data==3) break;}
		  print (8,2,"    ");}
	            
		  if (num_menu==2){		//Изменяем параметр №2
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) igain=igain+encoder_mul;
		  if (data==2) igain=igain-encoder_mul;
		  if (igain>limitIgain) igain=limitIgain;		 
		  if (igain<0) igain=0;
		  Hope_parameter(); 		  
		  print (2,2,con_int2strD((igain),5,0));
		  if (data==3) break;}
		  print (8,2,"    ");}
		
		  if (num_menu==3){		//Изменяем параметр №3
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) dgain=dgain+encoder_mul;
		  if (data==2) dgain=dgain-encoder_mul;
		  if (dgain>limitDgain) dgain=limitDgain;		 
		  if (dgain<0) dgain=0;
		  Hope_parameter(); 		  
		  print (2,2,con_int2strD((dgain),5,0));
		  if (data==3) break;}
		  print (8,2,"    ");}
		
		  if (num_menu==4){		//Изменяем параметр №4
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) PSK_gain=PSK_gain+encoder_mul;
		  if (data==2) PSK_gain=PSK_gain-encoder_mul;
		  if (PSK_gain>limitPWMfreqH) PSK_gain=limitPWMfreqH;		 
		  if (PSK_gain<limitPWMfreqL) PSK_gain=limitPWMfreqL; 		  
		  print (2,2,con_int2strD((PSK_gain),5,0));
		  TIM2->PSC=PSK_gain;
		  if (data==3) break;}
		  print (8,2,"    ");}
		  
		  if (num_menu==5){		//Изменяем параметр №5
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) Servo_time=Servo_time+encoder_mul;
		  if (data==2) Servo_time=Servo_time-encoder_mul;
		  if (Servo_time>limitServoTimeH) Servo_time=limitServoTimeH;		 
		  if (Servo_time<limitServoTimeL) Servo_time=limitServoTimeL; 		  
		  print (2,2,con_int2strD((Servo_time),5,0));
		  Hope_parameter();
		  if (data==3) break;}
		  print (8,2,"    ");}
		  
		  if (num_menu==6){		//Изменяем параметр №6
		  print (8,2,"Edit");
		  while(1){	
	            data=encoder();
		  if (data==1) limitERROR=limitERROR+encoder_mul;
		  if (data==2) limitERROR=limitERROR-encoder_mul;
		  if (limitERROR>limitErrorH) limitERROR=limitErrorH;		 
		  if (limitERROR<limitErrorL) limitERROR=limitErrorL; 		  
		  print (2,2,con_int2strD((limitERROR),5,0));
		  if (data==3) break;}
		  print (8,2,"    ");}

		  if (num_menu==7){		//Сохранение
		  uint8_t code=0;		    
		  print (6,2,"CODE");
		  while(1){	
	            data=encoder();
		  if (data==1) code=code+(uint8_t)encoder_mul;
		  if (data==2) code=code-(uint8_t)encoder_mul;
		  print (2,2,con_int2strD((code),3,0));
		  if (data==3) break;}
		  if (data==savecode) SAVE();
		  data=VERIFY();
		  if (data==0) print (0,2,"SAVE OK   ");
		     else print (0,2,"SAVE ERROR");
		  while(1){	            //типа паузы
	            data=encoder();
		  if (data==3) break;}		   
		  print (6,2,"    ");}
	
}}
		CLS();}

//+++++ len длинна строки +++++ 
unsigned char strLen (char * string){
        char cnt = 0;
        while (1){
	if (string[cnt]==0) break;	
	cnt++;}
        return cnt;}

uint8_t number_size(uint32_t number){
	uint8_t result = 0;
	if (number > 0) {
		while (number != 0) {
			number /= 10;
			result++;
		}
	} else {
		result = 1;
	}
	return result;}

//+++++ CONVERT FLOAT2STRING +++++
char *con_float2str(float Value, char Accuracy){
          char str_count=0;  //счетчик числа в массиве
	unsigned char Len;
	uint32_t div;
	
	if (Value<0){ 
          string[str_count]='-';
	str_count++;
          Value = -Value;}

	uint32_t integer=(unsigned long) Value;	
	uint32_t fractional=(uint64_t)((Value - integer) * pow(10,Accuracy));;
          uint8_t size = number_size(integer);
	
	for (uint8_t i = 1; i <= size; i++) {
	div = pow(10,size-i);
	string[str_count] = (uint32_t)(integer/div)+0x30;
	str_count++;
	integer = integer % div;}
	string[str_count]=0;
	
	Len=strLen(string);
	
	if (Accuracy > 0) {
	string[Len++]='.';
	for (uint8_t i = 1; i <= Accuracy; i++) {
	div = pow(10, Accuracy - i);
	string[Len] = (uint32_t)(fractional/div)+0x30;
	Len++;
	fractional = fractional % div;}
          string[Len]=0;
	while(1){
	if (string[Len]>0x30||string[Len]=='.') break; 
	string[Len]=0; 
	Len--;}	
	}
	Len++;	
	string[Len]=0;	
	return string;}

//+++++ Input FLOAT +++++
float get_param2(){
   	float data=0;
	char a;
	char b;
	uint32_t count;	
         if (readcount==2) return Err;	
         if (readcount>9) return Err;
 
         count=readcount-2;			// Находим . или ,
         while(1){
	 if ((dataread[count]==0x2e)||(dataread[count]==0x2c)){ 
	b=count-1;                                //указатель на целую часть
	a=count+1;                                //указатель на дробную часть	  
	break;}
         if (count<1) {
	a=readcount-1;
	b=readcount-2;
	break;}
         count--;}

         count=0;
         while(1){
         uint8_t num=get_mass_num(b);             //насчитываем целую часть
         if (num==0xff) break;
         uint32_t add_num=pow(10,count);
         count++;
         b--;
         for (uint8_t st=0; st<num; st++){
	 data+=add_num;}}
         count=1;
         while(1){
         uint8_t num=get_mass_num(a);             //насчитываем дробную часть
         if (num==0xff) break;
         uint32_t add_num=pow(10,count);
         count++;
         a++;
         double add_numD=1/(float)add_num;
         for (uint8_t st=0; st<num; st++){
	 data+=(float)add_numD;}         
         }
         if (dataread[1]==0x2d)data=-data;
         return data;}

void EXTI9_5_IRQHandler(void){
    EXTI->PR |= EXTI_PR_PR9;  
	last_error_SD=TIM1->CNT;  
	last_error_SD3++;
    if ((GPIOA->IDR&GPIO_IDR_IDR9)==0) {TIM1->CR1&=~TIM_CR1_DIR;}  
    else {TIM1->CR1|=TIM_CR1_DIR;}
}

//+++++ Прямое открытие канала +++++
uint8_t open_channel(uint8_t channel){
    if (channel>0x30) channel-=0x30;
    if (channel>4) return 0xff;
    RCC->APB1ENR&=~RCC_APB1ENR_WWDGEN;
	NVIC_DisableIRQ(WWDG_IRQn);
    NVIC_DisableIRQ(USART3_IRQn);
    NVIC_DisableIRQ (EXTI9_5_IRQn);
    for (uint32_t a=0; a<test_count; a++){
    systick=test_time_open;
    if (channel==1) TIM2->CCR1=test_zapoln;
    if (channel==2) TIM2->CCR2=test_zapoln;    
    if (channel==3) TIM2->CCR3=test_zapoln;    
    if (channel==4) TIM2->CCR4=test_zapoln;    
    while(systick){}    
    TIM2->CCR1=0;
    TIM2->CCR2=0;	
    TIM2->CCR3=0;	
    TIM2->CCR4=0;
    systick=test_time_count;
    while(systick){}}
    NVIC_EnableIRQ (EXTI9_5_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(WWDG_IRQn);	
  	RCC->APB1ENR|=RCC_APB1ENR_WWDGEN;   
    return channel;}

//+++++ Выключение всех графиков +++++
void plot_off(void){
    FLAG&=~2;
    FLAG&=~32;
    FLAG&=~64;
    FLAG2&=~1;  
}

#define MAXDEVICES 10           //Максимальное количество устройств на шине
#define	OW_SEARCH_FIRST	0xFF    //start new search
#define	OW_PRESENCE_ERR	0xFF
#define	OW_DATA_ERR	    0xFE
#define OW_LAST_DEVICE	0x00    //last device found
#define OW_ROMCODE_SIZE	8       //размер ROM включая CRC
uint8_t owDevicesIDs[MAXDEVICES][OW_ROMCODE_SIZE];	// Их ID
float termo[MAXDEVICES];        //Массив для хранения данных температуры
//+++++ Запись состояния 0 или 1 +++++
void OW_Write(uint8_t a){
    if (a==1) GPIOA->BSRR|=(1<<4);
    else
      GPIOA->BSRR|=(1<<20);}                                                                           

//+++++ Чтение состояния ножки датчика +++++
uint8_t OW_Read(void){
return GPIOA->IDR&GPIO_IDR_IDR4;}   

#define time_reset 6000
#define time_wait 1000
#define pr_pulse 200
//+++++ Сброс и чтение импульса присутствия +++++
uint8_t OW_Reset(void){                  
          OW_Write(0);                   //RESET & PRESENCE PULSE DALAS
          pause (time_reset);
          OW_Write(1);
          
          uint16_t temp=time_wait;
          uint16_t temp2=0;
          while(temp){
          uint8_t data=OW_Read();  
          if (data==0) temp2++;  
          temp--;}
          if (temp2>pr_pulse) return 0;
          return 1;}

#define time_dalas0 800
#define time_dalas1 30
#define time_dalasRES 25
//+++++ Запись бита 0 или 1 +++++
void OW_WriteBit(uint8_t a){

    if (a==0) {OW_Write(0);
               pause(time_dalas0);
               OW_Write(1);}
    
    if (a==1) {OW_Write(0);
               pause(time_dalas1);
               OW_Write(1);
               pause (time_dalas0-time_dalas1);}
               pause (time_dalasRES);}
  
//+++++ Запись байта +++++
void OW_WriteByte(uint8_t a){
    for(uint8_t i=0; i<8; i++) {
        OW_WriteBit(a&1);
        a>>=1;}}   

#define time_waitR 120
#define time_1pulse 10
//+++++ Чтение бита +++++
uint8_t OW_ReadBit(void){
    OW_Write(0);
    pause(time_dalas1);
    OW_Write(1);
    uint8_t temp2=0;
    uint8_t temp=time_waitR;
    while(temp){
    uint8_t data=OW_Read();  
    if (data==0) temp2++;  
    temp--;}  
    if (temp2>time_1pulse) return 0;  
    return 1;}

//+++++ Чтение байта +++++ 
uint8_t OW_ReadByte(void){
    uint8_t res=0;
    for(uint8_t a=0; a<8; a++){
        res>>=1;
        res|=OW_ReadBit()<<7;}
    return res;}

//+++++ Чтение ROM [33h] +++++   
uint8_t dalasROM(void){
        uint8_t data,crc=0;
        data=OW_Reset();
        if (data) return 1;
        OW_WriteByte(0x33);
        
        //чтение блока данных
        for (uint8_t a=0; a<7; a++){
        data=OW_ReadByte();
         owDevicesIDs[0][a]=data;

       //вычисление CRC - обрабатываем каждый бит принятого байта
        for(uint8_t j=0; j<8; j++){          
        uint8_t tmp = (crc ^ data) & 0x01;
        if (tmp==0x01) crc = crc ^ 0x18;
        crc = (crc >> 1) & 0x7F;
        if (tmp==0x01) crc = crc | 0x80;
        data = data >> 1;}}        
        data=OW_ReadByte();
        owDevicesIDs[0][7]=data;
        // 0 - Чтение успешно 1 - ОШИБКА
        if (crc=data) return 0;
        return 1;}

uint8_t OW_SearchROM( unsigned char diff, unsigned char *id ){
 	
	unsigned char i, j, next_diff;
	unsigned char b;

	if(OW_Reset()) 
		return OW_PRESENCE_ERR;       // error, no device found

	OW_WriteByte(0xF0);     // ROM search command
	next_diff = OW_LAST_DEVICE;      // unchanged on last device
	
	i = OW_ROMCODE_SIZE * 8;         // 8 bytes
	do 
	{	
		j = 8;                        // 8 bits
		do 
		{ 
			b = OW_ReadBit();			// read bit
			if( OW_ReadBit() ) 
			{ // read complement bit
				if( b )                 // 11
				return OW_DATA_ERR;  // data error
			}
			else 
			{ 
				if( !b ) { // 00 = 2 devices
				if( diff > i || ((*id & 1) && diff != i) ) { 
						b = 1;               // now 1
						next_diff = i;       // next pass 0
					}
				}
			}
            OW_WriteBit( b );               // write bit
         *id >>= 1;
         if( b ) *id |= 0x80;			// store bit
         i--;
		} 
		while( --j );
		id++;                            // next byte
    } 
	while( i );
	return next_diff;                  // to continue search
}

void OW_FindROM(unsigned char *diff, unsigned char id[]){

	while(1)
    {
		*diff = OW_SearchROM( *diff, &id[0] );
    	if ( *diff==OW_PRESENCE_ERR || *diff==OW_DATA_ERR ||
    		*diff == OW_LAST_DEVICE ) return;
		return;
    }
}

//+++++ поиск всех устройств на шине +++++ 
uint8_t search_ow_devices(void){
 
	unsigned char	i;
   	unsigned char	id[OW_ROMCODE_SIZE];
   	unsigned char	diff, sensors_count;

	sensors_count = 0;

	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && sensors_count < MAXDEVICES ; )
    { 
		OW_FindROM( &diff, &id[0] );

      	if( diff == OW_PRESENCE_ERR ) break;

      	if( diff == OW_DATA_ERR )	break;

      	for (i=0;i<OW_ROMCODE_SIZE;i++)
         	owDevicesIDs[sensors_count][i] = id[i];
		
		sensors_count++;
    }
	return sensors_count;}

//+++++ Выбор ROM +++++
uint8_t OW_MatchROM(unsigned char *rom){
 	if (OW_Reset()) return 1;
	OW_WriteByte(0x55);	
	for(unsigned char i=0; i<8; i++)
	{
	OW_WriteByte(rom[i]);
	}
 return 0;}

//+++++ запуск измерения температуры +++++
uint8_t StartMeasure(uint8_t* rom){
	//Reset, skip ROM and start temperature conversion
	if (OW_Reset()) return 1;
	
	if (rom) OW_MatchROM(rom);
	else OW_WriteByte(0xCC);
	OW_WriteByte(0x44);
	return 0;}

#define CRC8INIT	0x00
#define CRC8POLY	0x18              //0X18 = X^8+X^5+X^4+X^0
uint8_t crc8(uint8_t *data_in, unsigned int number_of_bytes_to_read ){

	uint8_t	crc;
	unsigned int	loop_count;
	uint8_t	bit_counter;
	uint8_t	data;
	uint8_t	feedback_bit;

	crc = CRC8INIT;
	
	for (loop_count = 0; loop_count != number_of_bytes_to_read; loop_count++)
	{ 
		data = data_in[loop_count];

		bit_counter = 8;
		do { 
			feedback_bit = (crc ^ data) & 0x01;
			if (feedback_bit==0x01) crc = crc ^ CRC8POLY;

			crc = (crc >> 1) & 0x7F;
			if (feedback_bit==0x01) crc = crc | 0x80;

			data = data >> 1;
			bit_counter--;
		}
		while (bit_counter > 0);
	}
	return crc; }

uint8_t ReadData(uint8_t *rom, uint8_t *buffer){

	//Reset, skip ROM and send command to read Scratchpad
	if (OW_Reset()) return 1;
	
	if (rom) OW_MatchROM(rom);
	else OW_WriteByte(0xCC);
	
	OW_WriteByte(0xbe);
	
	for (uint8_t i=0; i<9; i++) buffer[i] = OW_ReadByte();

	if (crc8(buffer, 9)) return 1;	// если контрольная сумма не совпала, возвращаем ошибку

	return 0;}

float ConvertToThemperature(uint8_t *buffer){
        uint8_t temp=buffer[0];
        uint8_t temp2=buffer[1];
        int16_t data=(int16_t)(temp|temp2<<8); 
        float term=(float)data/16;  
        return term;}

//+++++ ОШИБКА +++++
void S_ERROR(void){
        SERROR(1);
		if ((FLAG&0x80)==0){
                  for (uint8_t a=0; a<numID; a++){
          USART_Send(0x0d);	    
	      USART_COMM("Termo");
          USART_COMM(con_int2strD((a),0,0));
	      USART_COMM(" ");          
          USART_COMM(con_float2str((termo[a]),6));
	      USART_COMM("C");}  
       	  USART_Send(0x0d);	      
	      USART_COMM("Position Error ");
	      USART_COMM(con_int2strD(((long)counter_step),0,0));   
		USART_Send(0x0d);
		USART_COMM("ERROR");		
		TIM2->CCER=0x00000000;
		TIM2->CR1&=~TIM_CR1_CEN;
	          TIM2->CCR1=0;
	          TIM2->CCR2=0;	
	          TIM2->CCR3=0;	
	          TIM2->CCR4=0;
		FLAG|=0x80;}} 

void DS18TERMO(void){
        termo_count++;
        if (termo_count==limitTermo1){
        StartMeasure(owDevicesIDs[numID_count]);}      // запускаем измерение  
          
        if (termo_count==limitTermo2){          
        uint8_t temp=ReadData(owDevicesIDs[numID_count],DS18data);
        if (!temp) termo[numID_count]=ConvertToThemperature(DS18data);  
        numID_count++;
        if (numID_count==numID) numID_count=0;
        termo_count=0;
        // тут управляем выходом T2 для охлаждения радиатора
        if (termo[0]>t_on) COOLER(0); //включить охлаждение 
        if (termo[0]<t_off) COOLER(1); //выключить охлаждение         
        if (termo[0]>=t_error1) S_ERROR();
        if (termo[1]>=t_error2) S_ERROR(); 
}}

 
void main(void){	
	RCC->CSR|=RCC_CSR_RMVF;		            // Сброс ресет флагов
	RCC->APB1ENR|=RCC_APB1ENR_PWREN;        // Включить силу и мощь
	RCC->APB1ENR|=RCC_APB1ENR_BKPEN;	
	PWR->CR|=0x000001F0;
		
//	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN){  
//	RCC->BDCR|=RCC_BDCR_LSEON;		// НАСТРАИВАЕМ ЧАСЫ
//	while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0){}		
//          RCC->BDCR|=RCC_BDCR_RTCEN;
//          RCC->BDCR|=RCC_BDCR_RTCSEL_LSE;}
	
	LOAD();	
	    
  	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;	    // Подключение тактового сигнала A
	GPIOA->CRL=0x00559999;                  // Настраиваем ноги порта A
	GPIOA->CRH=0x88888844;
    GPIOA->ODR=0x0000bc00;   
    
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;	    // Подключение такта таймера №1
	
// настройки если подключен энкодер	
//	TIM1->SMCR|=0X00000002;                 // Таймер №1 в энкодер моде
//	TIM1->CR1|=TIM_CR1_CEN;                 // Включить таймер

// настройки если подключен STEP/DIR
    TIM1->SMCR|=0X00000057;
	TIM1->CCMR1|=0x000000C1;
	TIM1->CR1|=TIM_CR1_CEN;                 // Включить таймер
    EXTI->FTSR |= EXTI_FTSR_TR9;
    EXTI->RTSR |= EXTI_RTSR_TR9;
	EXTI->IMR |= EXTI_IMR_MR9;
    NVIC_EnableIRQ (EXTI9_5_IRQn);          // Это прерывание прописано в SYSTICK!!!!
	NVIC_SetPriority(EXTI9_5_IRQn,1);	
	
    RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;	    // Подключение такта таймера №3
	TIM3->SMCR|=encoderM;     	            // Таймер №3 в энкодер моде
	TIM3->CR1|=TIM_CR1_CEN;                 // Включить таймер

// настройки если подлючен энкодер второго двигателя	
    RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;	    // Подключение такта таймера №4
	TIM4->SMCR|=encoderM;     	            // Таймер №4 в энкодер моде
	TIM4->CR1|=TIM_CR1_CEN;                 // Включить таймер
	
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;       // Подключение тактового сигнала B
	GPIOB->CRL=0x88882422;                  // Настраиваем ноги порта B
	GPIOB->CRH=0x22224A22;
	RCC->APB2ENR|=RCC_APB2ENR_AFIOEN;       // РЕМАП PB4, PB5 то TIM3
	AFIO->MAPR|=AFIO_MAPR_TIM3_REMAP_PARTIALREMAP;
	AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	
// настройки если подключен энкодер на управление на TIM4	
//  RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;	    // Подключение такта таймера №4
//	TIM4->SMCR|=0X00000002;                 // Таймер №4 в энкодер моде
//	TIM4->CCMR1=0x0000F1F1;
//	TIM4->CCER=0x00000022;
//	TIM4->CNT=0x8000;
//	TIM4->CR1|=TIM_CR1_CEN;                 // Включить таймер
	
    RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;	    // Подключение такта таймера №2	
    TIM2->CR1=0x00000280;
	TIM2->CCMR1=0x00006060;
	TIM2->CCMR2=0x00006060;
	TIM2->CCER=0x00001111;
	TIM2->ARR=0X1FF;
	TIM2->PSC=PSK_gain;
	TIM2->CR1|=TIM_CR1_CEN;                 // Включить таймер	
	
 	RCC->APB2ENR|=RCC_APB2ENR_IOPCEN;	    // Подключение тактового сигнала C
	GPIOC->CRH=0x44644444;                  // Настраиваем ноги порта С
	
	DBGMCU->CR|=DBGMCU_CR_DBG_WWDG_STOP;	//СТОП ТАЙМЕРОВ ПРИ ОТЛАДКЕ
	DBGMCU->CR|=DBGMCU_CR_DBG_IWDG_STOP;

	RCC->APB1ENR|=RCC_APB1ENR_WWDGEN;	    //включить тактирование WWDG		
	WWDG->CR|=0xFF;			                //Разрешить работу и установить начальное значение
  	WWDG->CFR|=0x101;
	WWDG->SR&=~WWDG_SR_EWIF;		        //Сброс флага
    WWDG->CFR|=WWDG_CFR_EWI;	

    RCC->APB1ENR|=RCC_APB1ENR_USART3EN;
	USART3->BRR=brr_calc(38400);            //рассчет делителя	
	USART3->CR1|=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
	USART3->CR1|=USART_CR1_RXNEIE;

    RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;                       // Тактируем АЦП
    ADC1->CR2 |= ADC_CR2_ADON;                              // включить АЦП
    ADC1->CR2 |= ADC_CR2_CAL;                               // запуск калибровки
    while (!(ADC1->CR2 & ADC_CR2_CAL)){};                   // ждем окончания калибровки
    ADC1->CR2=ADC_CR2_EXTSEL;                               // выбрать источником запуска разряд  SWSTART    
    ADC1->CR2|=ADC_CR2_EXTTRIG;                             // разр. внешний запуск регулярного канала
    ADC1->CR2|=  ADC_CR2_CONT;                              // режим непрерывного преобразования
    ADC1->SQR3=7;                                           // загрузить номер канала
    ADC1->SMPR2=(7<<21);                                    // время выборки
    ADC1->CR1|=ADC_CR1_EOCIE;                               // включить прерывания
    ADC1->CR2 |= ADC_CR2_ADON;                              // включить АЦП
    ADC1->CR2 |= ADC_CR2_SWSTART;  

    NVIC_EnableIRQ(ADC1_2_IRQn);            //прерывание АЦП
    NVIC_SetPriority(ADC1_2_IRQn,3);    
	NVIC_EnableIRQ(USART3_IRQn);            //разрешение прерывания USART
	NVIC_SetPriority(USART3_IRQn,2);
	NVIC_SetPriority(SysTick_IRQn,2);
	NVIC_EnableIRQ(WWDG_IRQn);              //Инициализация прерывания WWDG
	NVIC_SetPriority(WWDG_IRQn,1);	
	__enable_irq();
	EXTI9_5_IRQHandler();
    
	iniciale();			                    // Инициализация LCD	
	
	FLASH->ACR|=0x00000002;
	RCC->CR|=RCC_CR_HSEON; 
	while ((RCC->CR & RCC_CR_HSERDY)==0) {} // Ожидание готовности	  	  	
	RCC->CFGR=0x0025E400;	
	RCC->CR|=(1<<24);         		        //PLL ON
	while ((RCC->CR & RCC_CR_PLLRDY)==0) {} // Ожидание готовности 
	RCC->CFGR|=(1<<1);        		        //SWITCH PLL
	RCC->CR&=~RCC_CR_HSION;
    
    numID=search_ow_devices();      // определяем количество датчиков и читаем ROMы
    COOLER(1);
    OW_Reset();
    OW_WriteByte(0xCC);
    OW_WriteByte(0x44);
    while(!OW_ReadBit()){}
    for (uint8_t a=0; a<numID; a++){
    ReadData(owDevicesIDs[numID],DS18data);
    termo[a]=0;}


	
	print (1,1,"KURKOV-REDUKTORS");
	print (1,2,"+79-26-26-53-514");    
    
	USART_Send(0x0d);	            // кто сотрет это, тот нарушает права разработчика
	USART_COMM("KURKOV-REDUKTORS"); // мало того, разработчик может узнать и подать в суд 
	USART_Send(0x0d);	            // плюс, разработчик дытына свыше сотни кил
	USART_COMM("+79-26-26-53-514"); // может просто залепить в ухо и будет прав
	USART_Send(0x0d);               // нехуй нарушать права!!!!!!!!!!!!
	USART_COMM(">");
	
	Hope_parameter();
 
    
	while (1){
    long data=TIM2->CCR1;
    data=data+TIM2->CCR2;
	if (data>limitWARN) WARN(0);
	else WARN(1);
	  
    data=0-limitRUN;
	if (counter_step>limitRUN|counter_step<data) BUSY(1);
	else BUSY(0);
	
    data=0-limitERROR;
	if (counter_step>limitERROR|counter_step<data) S_ERROR();
    
//	print (1,1,"+++++ERROR+++++ ");
//	print (1,2,"   OVER RUN     ");
		
//	print (1,1,con_int2strD((positionA),0,0));
//	print (1,2,con_int2strD((positionB),0,0));
//	print (11,1,con_float2str(counter_step,3));	
//	print (11,2,con_int2strD((uint32_t)(counter_step),0,0));
//	data=encoder();
//	if (data==1) counter_step+=encoder_mul;
//	if (data==2) counter_step-=encoder_mul;	
//	if (data==3) menu();

        if ((FLAG&1)==1){                        
    if (dataread[0]==0x41){               // A - вкл график
          data=get_param();
	      USART_Send(0x0d);
          USART_COMM("1-Error 2-Force 3-Current ");
          USART_COMM("4-Current/10");          
	      USART_Send(0x0d);
          plot_off();
          if (data==1){
          FLAG|=2;
          USART_COMM("PLOT ERROR ON");}     // 1-график ошибки
          if (data==2){
          FLAG|=32;
          USART_COMM("PLOT FORCE ON");}     // 2-график усилия
          if (data==3){
          FLAG|=64;
          USART_COMM("PLOT CURRENT ON");}   // 3-график тока
          if (data==4){
          FLAG2|=1;
          USART_COMM("PLOT CURRENT/10 ON");}// 4-график тока/10          
            }	                           
    if (dataread[0]==0x58){              // X - выкл график
	      USART_Send(0x0d);
	      USART_COMM("PLOT OFF");
	      plot_off();}
    if (dataread[0]==0x52){              // R - Reset Soft R+ - Reset Hard
	      if (dataread[1]==0x2b) WWDG->CR=0X00;
	      else{
		  TIM3->CNT=0;
		  TIM1->CNT=0;
		  counter_step=0;
		  SERROR(0);
		  Reset_Handler();}}
    if (dataread[0]==0x4c){               // L - загрузить переменные
	      USART_Send(0x0d);
	      USART_COMM("Gain Load");
	      LOAD();}
    if (dataread[0]==0x53){               // S - сохранить переменные
          data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("Gain Save");
	      USART_Send(0x0d);	    
	      if (data==savecode) SAVE();
	      data=VERIFY();
	      if (data==0) USART_COMM("SAVE OK");
	      else USART_COMM("SAVE ERROR");}	    
    if (dataread[0]==0x50){               // P - Pgain
          data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("P GAIN  ");
	      if (data==Err) data=pgain;  
	      if (data>limitPgain) data=limitPgain;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      pgain=(uint16_t)data;
	      Hope_parameter();}
	  if (dataread[0]==0x49){               // I - Igain	    
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("I GAIN  ");
	      if (data==Err) data=igain;  
	      if (data>limitIgain) data=limitIgain;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");	      
	      igain=(uint16_t)data;
	      Hope_parameter();}	  
	  if (dataread[0]==0x44){               // D - Dgain
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("D GAIN  ");
	      if (data==Err) data=dgain;  
	      if (data>limitDgain) data=limitDgain;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      dgain=(uint16_t)data;
	      Hope_parameter();}
	  if (dataread[0]=='V'){               // V - Vgain
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("V GAIN  ");
	      if (data==Err) data=vgain;  
	      if (data>limitVgain) data=limitVgain;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      vgain=(uint16_t)data;
	      Hope_parameter();}     
	  if (dataread[0]=='O'){               // O - Current
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("Current  ");
	      if (data==Err) data=current_max;  
	      if (data>limitCurrent) data=limitCurrent;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      current_max=(uint16_t)data;}
    	  if (dataread[0]==0x46){               // F - ERROR
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("Fault  ");
	      if (data==Err) data=limitERROR;  
	      if (data>limitErrorH) data=limitErrorH;
	      if (data<limitErrorL) data=limitErrorL;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      limitERROR=(uint16_t)data;}
	  if (dataread[0]==0x59){               // Y - Servo cycle
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("Servo cycle  ");
	      if (data==Err) data=Servo_time;  
	      if (data>limitServoTimeH) data=limitServoTimeH;
	      if (data<limitServoTimeL) data=limitServoTimeL;
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      Servo_time=(uint16_t)data;
	      data=freq/Servo_time;
	      USART_Send(0x0d);
	      USART_COMM(con_int2strD((data),0,0));	      
	      USART_COMM("Hz");
	      Hope_parameter();}
	  if (dataread[0]==0x57){               // W - PWM freq
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("PWM freq  ");
	      if (data==Err) data=PSK_gain;  
	      if (data>limitPWMfreqH) data=limitPWMfreqH;
	      if (data<limitPWMfreqL) data=limitPWMfreqL;	      
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      PSK_gain=(uint16_t)data;
	      data=freq/(PSK_gain*TIM2->ARR);
	      USART_Send(0x0d);
	      USART_COMM(con_int2strD((data),0,0));	      
	      USART_COMM("Hz");
	      TIM2->PSC=PSK_gain;} 
	  if (dataread[0]=='U'){               // U - PWM max
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("PWM max  ");
	      if (data==Err) data=PWMmax;  
	      if (data>limitPWMmaxH) data=limitPWMmaxH;
	      if (data<limitPWMmaxL) data=limitPWMmaxL;
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(":");
	      PWMmax=(uint16_t)data;}	  
	  if (dataread[0]==0x4d){               // M - MulDiv
	      float data2=get_param2();
	      USART_Send(0x0d);
	      USART_COMM("MulDiv  ");
	      if (data==Err) data2=muldiv;
	      muldiv=data2;  
	      if(muldiv<(1/limitMuldiv)|muldiv>limitMuldiv)muldiv=1;	      
          USART_COMM(con_float2str((muldiv),6));
	      USART_COMM(":");}
	  if (dataread[0]=='E'){                // E - Encoder Mode
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("Encoder Mode  ");
	      if (data==Err) data=encoderM; 
	      if((data<1)||(data>3))data=1;
	      encoderM=data;
          USART_COMM(con_int2strD((encoderM),1,0));
	      TIM3->SMCR&=TIM_SMCR_SMS;
	      TIM3->SMCR|=encoderM;     	// Таймер №3 в энкодер моде
	      USART_COMM(":");}
	  if (dataread[0]=='T'){                // T - Dead Time
	      data=get_param();
	      USART_Send(0x0d);
	      USART_COMM("DeadTime  ");
	      if (data==Err) data=deadtime; 
	      if((data<1)||(data>limitdeadtime))data=deadtimebase;
	      deadtime=data;	      
          USART_COMM(con_int2strD((deadtime),0,0));
	      USART_COMM(":");}
	  if (dataread[0]==0x51){               // Q - Status
	      USART_Send(0x0d);	    
	      USART_COMM("PositionA  ");
	      USART_COMM(con_int2strD((positionA),0,0));	      
	      USART_Send(0x0d);
	      USART_COMM("PositionB  ");
	      USART_COMM(con_int2strD((positionB),0,0));	      
	      USART_Send(0x0d);	      
	      USART_COMM("Position Error ");
	      USART_COMM(con_int2strD(((long)counter_step),0,0));}	    	  
	  if (dataread[0]=='G'){                // G - Go
	      float data2=get_param2();
	      USART_Send(0x0d);	    
	      USART_COMM("GO!!! ");
	      if (data==Err) data2=0;
	      USART_COMM(con_float2str((data2),2));	      
          counter_step+=data2;
	      USART_COMM(" Step(s)");}	  
	  if (dataread[0]=='C'){                // C - Test
	      USART_Send(0x0d);
          USART_COMM("1-SHOW DATA 2-PSK ON/OFF ");
          USART_COMM("3-TEST DATA 4-RUN TEST");          
	      USART_Send(0x0d);
          USART_COMM("5-SHOW TEMP 6-SET TEMP");          
	      USART_Send(0x0d);

          if (dataread[1]=='6'){  //ввод параметров температурных параметров         	    
	      USART_COMM("Input temp parameters");
	      USART_Send(0x0d);	    
	      USART_COMM("C6 P1 P2 P3 P4");              
          get_param_temp();
	      USART_Send(0x0d);	    
	      USART_COMM("P1 T Cooler ON ");          
	      USART_COMM(con_int2strD((t_on),0,0));
          USART_COMM("C");
	      USART_Send(0x0d);	    
	      USART_COMM("P2 T Cooler OFF ");
	      USART_COMM(con_int2strD((t_off),0,0));
          USART_COMM("C");
	      USART_Send(0x0d);	    
	      USART_COMM("P3 T Error 0 ");
	      USART_COMM(con_int2strD((t_error1),0,0));
          USART_COMM("C");
	      USART_Send(0x0d);	    
	      USART_COMM("P4 T Error 1 ");
	      USART_COMM(con_int2strD((t_error2),0,0));
          USART_COMM("C");}          
          
	      if (dataread[1]=='5'){  //вывод значений датчиков температуры
          for (uint8_t a=0; a<numID; a++){
          USART_Send(0x0d);	    
	      USART_COMM("Termo");
          USART_COMM(con_int2strD((a),0,0));
	      USART_COMM(" ");          
          USART_COMM(con_float2str((termo[a]),6));
	      USART_COMM("C");}}
                      
          
	      if (dataread[1]=='1'){  //тут типа вкл показаний перемещений при прерывании
	      if ((FLAG&4)==0){
	      USART_COMM("Test ON");
	      FLAG|=4;}
	      else{
	      USART_COMM("Test OFF");	        
	      FLAG&=~4;}}
	  
	      if (dataread[1]=='2'){  //тут типа вкл зависимости частоты шим от силы
	      if ((FLAG&8)==0){
	      USART_COMM("PSK ON");
	      FLAG|=8;}
	      else{
	      USART_COMM("PSK OFF");	        
	      FLAG&=~8;}}
          
          if (dataread[1]=='3'){  //ввод параметров токового теста им.Куркова          	    
	      USART_COMM("Input test parameters");
	      USART_Send(0x0d);	    
	      USART_COMM("C3 P1 P2 P3 P4");              
          get_param_test();
	      USART_Send(0x0d);	    
	      USART_COMM("P1 Duty cycle ");          
	      USART_COMM(con_int2strD((test_zapoln),0,0));
	      USART_Send(0x0d);	    
	      USART_COMM("P2 Time open ");
	      USART_COMM(con_int2strD((test_time_open),0,0));
          USART_COMM(" x10 mkS");
	      USART_Send(0x0d);	    
	      USART_COMM("P3 Number of packs ");
	      USART_COMM(con_int2strD((test_count),0,0));
	      USART_Send(0x0d);	    
	      USART_COMM("P4 Time between packs ");
	      USART_COMM(con_int2strD((test_time_count),0,0));
          USART_COMM(" x10 mkS");}

          if (dataread[1]=='4'){  //открытие указанного канала согласно параметров
          WARN(0);
	      USART_COMM("++++ Attention +++");            
	      USART_Send(0x0d);            
	      USART_COMM("WARNING  Direct opening of the channel");
	      USART_Send(0x0d);            
	      USART_COMM("Be careful when working with the device");
          USART_Send(0x0d);
	      USART_COMM("Specify the channel number to open: 1 2 3 4");
          USART_Send(0x0d);
          
          FLAG|=16;                                         // вкл. метку
          SysTick_Config(880);
          TIM2->CCR1=0;
	      TIM2->CCR2=0;	
	      TIM2->CCR3=0;	
	      TIM2->CCR4=0;
          last_readcount=0;
          readcount=0;
          while (1){
          if (readcount!=last_readcount){
          TIM3->CNT=0;
          TIM4->CNT=0;              
          data=open_channel(dataread[0]);
          if (data==0xff) break;
	      USART_COMM("Open channel ");
	      USART_COMM(con_int2strD((data),0,0));
	      USART_COMM(" Ok  T3>");
	      USART_COMM(con_int2strD((TIM3->CNT),0,0));          
	      USART_COMM("  T4>");
	      USART_COMM(con_int2strD((TIM4->CNT),0,0));          
          USART_Send(0x0d);
          readcount=0;
          last_readcount=0;}}
          TIM3->CNT=0;                                     // при выходе с теста нужно все обнулить
          TIM4->CNT=0;
          TIM1->CNT=0;          
          counter_step=0;         
          last_encoderA=0;
          last_encoderB=0;
          positionA=0;
          positionB=0;
          last_error=0;       
          last_result=0;
	      USART_COMM("Closed");
          Hope_parameter();
          WARN(1);
          FLAG&=~16;}                                        // выкл.метку
          
	  	}
	  if (dataread[0]==0x23){               // # - Register	      
	      USART_Send(0x0d);
	      USART_COMM("P GAIN  ");
	      USART_COMM(con_int2strD((pgain),0,0));
	      USART_Send(0x0d);	      
	      USART_COMM("I GAIN  ");
	      USART_COMM(con_int2strD((igain),0,0));
	      USART_Send(0x0d);	      
	      USART_COMM("D GAIN  ");
	      USART_COMM(con_int2strD((dgain),0,0));
	      USART_Send(0x0d);
	      USART_COMM("V GAIN  ");
	      USART_COMM(con_int2strD((vgain),0,0));
	      USART_Send(0x0d);          
	      USART_COMM("Current  ");
	      USART_COMM(con_int2strD((current_max),0,0));
	      USART_Send(0x0d);
	      USART_COMM("Encoder Mode  ");
	      USART_COMM(con_int2strD((encoderM),1,0));
	      USART_Send(0x0d);	      
	      USART_COMM("Dead Time  ");
	      USART_COMM(con_int2strD((deadtime),0,0));	      
	      USART_Send(0x0d);	      
	      USART_COMM("Fault  ");   
	      USART_COMM(con_int2strD((limitERROR),0,0));	  
	      USART_Send(0x0d);	      
	      USART_COMM("Servo cycle  ");   
	      USART_COMM(con_int2strD((Servo_time),0,0));
	      USART_Send(0x0d);
	      USART_COMM("PWM freq  ");   
	      USART_COMM(con_int2strD((PSK_gain),0,0));
	      USART_Send(0x0d);
	      USART_COMM("PWM max  ");   
	      USART_COMM(con_int2strD((PWMmax),0,0));
	      USART_Send(0x0d);
	      USART_COMM("MulDiv  ");
          USART_COMM(con_float2str((muldiv),6));
                }
	  if (dataread[0]==0x48){               // H - Help
	      USART_Send(0x0d);	     
	      USART_COMM("Ax - Plot on");
	      USART_Send(0x0d);	     
	      USART_COMM("X  - Plot off");	     
	      USART_Send(0x0d);	     
	      USART_COMM("R  - Reset soft");
	      USART_Send(0x0d);	     
	      USART_COMM("R+ - Reset hard");
	      USART_Send(0x0d);
	      USART_COMM("#  - Variables List");
	      USART_Send(0x0d);	
	      USART_COMM("Px - Prop. const");
	      USART_Send(0x0d);	     
	      USART_COMM("Ix - Integ. const");
	      USART_Send(0x0d);	     
	      USART_COMM("Dx - Difer. const");
	      USART_Send(0x0d);
	      USART_COMM("Vx - Velocity. const");
	      USART_Send(0x0d);          
	      USART_COMM("Ox - Current max");
	      USART_Send(0x0d);           
	      USART_COMM("Fx - Fault. const");
	      USART_Send(0x0d);	     
	      USART_COMM("Mx - MulDiv const");	          
	      USART_Send(0x0d);
	      USART_COMM("Ex - Encoder Mode");	          
	      USART_Send(0x0d);
	      USART_COMM("Tx - Dead Time");	          	          
	      USART_Send(0x0d);	      
	      USART_COMM("Yx - Servo Time");	   
	      USART_Send(0x0d);	     
	      USART_COMM("Wx - PWM freq");
	      USART_Send(0x0d);
	      USART_COMM("Ux - PWM max");
	      USART_Send(0x0d);	  
	      USART_COMM("Q  - Global Position");
	      USART_Send(0x0d);	     
	      USART_COMM("L  - Load");
	      USART_Send(0x0d);	     
	      USART_COMM("Sx - Save");
	      USART_Send(0x0d);	     
	      USART_COMM("Gx - Go");
	      USART_Send(0x0d);	     
	      USART_COMM("C  - Test");
   
		}
	       	  
	FLAG&=~1;	  
    USART_Send(0x0d);
	USART_COMM(">");	
	readcount=0;}  

        if ((FLAG&1)==0){
	  if (readcount>0){	  
	     if (dataread[readcount-1]==0x08) {
	        dataread[readcount-1]=0;
          	USART_Send(0x0d);
		USART_COMM(">");
		last_readcount=0;  
		if (readcount>=2) readcount=readcount-2;}}
        if (readcount!=last_readcount){         // типа повторитель
	for (uint8_t a=last_readcount; a<readcount; a++){
 	     USART_Send(dataread[a]);}
	last_readcount=readcount;}}
	
        if ((FLAG&2)==2) {                 // График ошибки
	if (usart_graf_count>plot_freq) {graf((long)counter_step); usart_graf_count=0;}	  
	usart_graf_count++;}

        if ((FLAG&32)==32) {               // График усилия
        data=TIM2->CCR1;
        data=data-TIM2->CCR2;
	if (usart_graf_count>plot_freq) {graf(data>>2); usart_graf_count=0;}	  
	usart_graf_count++;}    

        if ((FLAG&64)==64) {               // График тока
    data=current;
	if (usart_graf_count>plot_freq) {graf(data); usart_graf_count=0;}	  
	usart_graf_count++;}

        if ((FLAG2&1)==1) {               // График тока/10
    data=current/10;
	if (usart_graf_count>plot_freq) {graf(data); usart_graf_count=0;}	  
	usart_graf_count++;}        
    
        if ((FLAG&4)==4){                  // Тест
	if (last_error_SD!=last_error_SD2){
	USART_Send(0x0d);  
	USART_COMM(con_int2strD((last_error_SD3),0,0));
	last_error_SD2=last_error_SD;
	USART_Send(' ');	
	USART_COMM(con_int2strD((positionB),0,0));
	USART_Send(' ');	
	USART_COMM(con_int2strD((positionA),0,0));	
	}}

        if ((FLAG&8)==8){                  // зависимости частоты ШИМ от силы нажатия
	uint16_t strong=TIM2->CCR1;
	strong+=TIM2->CCR2;
    strong=~strong;
    strong&=511;
    data=PSK_gain;
	if (strong>limitPWM2) data+=strong-limitPWM2;    
	if (strong>limitPWM1) data=(strong*3)-1200;    
	TIM2->PSC=data;} 
        else{
            TIM2->PSC=PSK_gain;}         
	
        if (acp1_count==0){                   // забрать данные с ацп напряжения
      current=acp1_data/acp1_con;
      if (current>acp1_null) current=acp1_null;
      current=acp1_null-current;
      if (current>current_max) {cur_err++;if (cur_err>PWMmax) cur_err=PWMmax;}
      else if (cur_err>0) cur_err--;
      PWM=PWMmax-cur_err;
      acp1_data=0;  
      acp1_count=acp1_con;}	

      DS18TERMO();

	
	
}
}


	      