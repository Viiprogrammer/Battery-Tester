/*
  Реализовать выбор напряжения оконьчания теста
  TP4056 зарядка, отслеживание зарядки по 2ум светодиодам 
   - пауза теста 
   - индикация нажатия кнопок если работает то зеленый если нет то красный (есть звуковая)
   - хардвар защита по температуре (есть софтвар)
   - Добавить защиту от переполюсовки, и индикацию неверного подключения
   - Прицепить энкодер
*/

#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

#include "config.h"
#include "buttons.h"
#include "usart.h"
#include "parser.h"

//LCD
#include "compilers_4.h"
#include "lcd_lib_2.h"

//Секунды
volatile unsigned long  seconds_timer2 = 0;

//Флаг прерывания
volatile bool interrupt_data = false;

//Флаг предварительной зарядки
bool charge_before = false;
/*
  Ток Емкость Ватт*часы
*/
unsigned long 
  LastCapacity = 0, 
  minutes = 0, 
  Capacity = 0,
  I = 0, 
  Voltage = 0,
  number_buffer = 0;
  
unsigned int 
  //Напряжение выключения
  END_Voltage = 2500,
  //Установленный ток
  I_set = 1000;
  
char i = 0;
char button_event = 0;
char one_char_buffer = 0;
char dialog_id = 0; 
uint16_t int_buffer = 0; 
uint8_t V_ = 0;
bool value_parsed_success = false;
bool pause_status = false;
//EEPROM
unsigned long eeLastCapacity EEMEM = 0;
unsigned long eeI EEMEM = 1000;
unsigned long eeEND_Voltage EEMEM = 2500;

const uint8_t leftArrow [] PROGMEM = {	
	0b00010,
	0b00100,
	0b01000,
	0b10000,
	0b01000,
	0b00100,
	0b00010,
	0b00000
};
 
const uint8_t rightArrow [] PROGMEM = {	
	0b01000,
	0b00100,
	0b00010,
	0b00001,
	0b00010,
	0b00100,
	0b01000,
	0b00000
};

uint32_t seconds() {
  uint32_t m;
  cli();
  m = seconds_timer2;
  sei();
  return m;
}


void PARS_Handler(uint8_t argc, char *argv[])
{
   int_buffer = PARS_StrToUint(argv[0]);
   if(dialog_id == VOLTAGE_DIALOG){
	   
	   if (int_buffer <= VOLTAGE_MAX && int_buffer >= VOLTAGE_MIN && int_buffer%VOLTAGE_STEP == 0)
	   {
		   USART_SendStr("OK\r\n");
		   value_parsed_success = true;
	   }else{
		   USART_SendStr("Invalid value\r\n");
	   }
   }
   if(dialog_id == AMPERAGE_DIALOG){
	   if (int_buffer <= AMPERAGE_MAX && int_buffer >= AMPERAGE_MIN && int_buffer%AMPERAGE_STEP == 0)
	   {
		   USART_SendStr("OK\r\n");
		   value_parsed_success = true;
	   }else{
		   USART_SendStr("Invalid value\r\n");
	   }
   }
}

void t2_init(){
   TIMSK &= ~(1 << OCIE2)|(1 << TOIE2);
   ASSR |= (1 << AS2);
   TCNT2 = 0;
   TCCR2 |= (1 << CS22)|(1 << CS20);
}


unsigned int read_adc(unsigned char adc_input)
{
  uint32_t adc_ = 0;
  ADMUX= adc_input | ADC_VREF_TYPE;
  _delay_us(10);
  for (char i = 0; i < ADC_READ_NUM; i++)
  {
	  ADCSRA|=(1<<ADSC);
	  while ((ADCSRA & (1<<ADIF))==0);
	  ADCSRA|=(1<<ADIF);
	  adc_ += ADCW;
  }	  
  return adc_/ADC_READ_NUM;
}

/*
  Подзалупные функции для экономии памяти 
*/
void printUL(char i)
{
    LCD_WriteData(0x30+i);
    USART_PutChar(0x30+i);
}
void printUARTLCD(char i, bool uart)
{
	LCD_WriteData(0x30+i);
	if(uart){
		USART_PutChar(0x30+i);
	}
}
void printITime(char a, char b)
{
    LCD_WriteData(0x30+a);
	LCD_WriteData(0x30+b);
    USART_PutChar(0x30+a);
    USART_PutChar(0x30+b);
}
/*
void printWhVoltage(unsigned long val, bool wh)
{
   if(wh){
     printUL(val/10000);
   }
   printUL((val%10000)/1000);
   LCD_WriteData('.');
   USART_PutChar('.');
   printUL((val%1000)/100);
   printUL((val%100)/10);
}*/
void printVoltage(unsigned long val)
{
	V_ = (val%100000)/10000;
	if(V_){printUL(V_);}
	printUL((val%10000)/1000);
	LCD_WriteData('.');
	USART_PutChar('.');
	printUL((val%1000)/100);
	if(!V_){
	 printUL((val%100)/10);
	}
}
/* Конец долбоебизма */

void checkBattery(bool clear, bool test)
{
   if(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(NO_BATTERY_VALUE)){
	 LCD_Clear();
     LCD_Goto(0,0);
     LCD_SendStr("Please connect");
	 LCD_Goto(0,1);
	 LCD_SendStr("the battery ");
	 OCR1A = 0;
	 if(test){
		 //Подключение АКБ
		 PORTB &=~ (1 << PB5);
		 //Включение таймера времени
		 TIMSK &= ~(1 << TOIE2);
	 }
	 while(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(NO_BATTERY_VALUE)){}
     if(test){
		 //Подключение АКБ
		 PORTB |= (1 << PB5);
		 //Включение таймера времени
		 TIMSK |= (1 << TOIE2);
		 //PWM Calc
		 OCR1A = 40*(I_set/100)+4*(I_set/100);
	 }
	 if(clear) LCD_Clear();
   }
}

void Reset_Button(){
	while(BUT_GetBut() != ENTER_BUTTON_ID && BUT_GetBut() == BUT_PRESSED_CODE){
		if(USART_GetChar()) break;
		BUT_Poll();
	}
}

void checkTempPotection(){
	if(read_adc(5) >= BATTERY_CRITICAL_TEMP_VALUE){
		 LCD_Clear();
		 LCD_Goto(0,0);
		 LCD_SendStr("High temperature");
		 LCD_Goto(1,1);
		 LCD_SendStr("Enter - reboot");
		 USART_SendStr("Critical temperarure!!! Test Stopped\r\n");
	     cli();
	     OCR1A = 0;
         PORTB &= ~(1 << PB5);
         TCCR1B &= ~(1 << CS11);	
         Reset_Button();
         USART_SendStr("Rebooting\r\n");
         soft_reset();
    }
}

void printCapacity(unsigned long Capacity, bool mode, bool uart){
	if(mode){
		LCD_Goto(4,1);
		if(uart){
			USART_SendStr("Capacity: ");
		}
	}
	
	//Десятки тыс
	printUARTLCD(Capacity/10000, uart);
	//Тысячи
	printUARTLCD((Capacity%10000)/1000, uart);
	//Сотни
	printUARTLCD((Capacity%1000)/100, uart);
	//Десятки
	printUARTLCD((Capacity%100)/10, uart);
	printUARTLCD((Capacity%10), uart);
	
	if(mode){
		if(uart){
			USART_SendStr(" mAh\r\n");
		}
		LCD_SendStr("mAh");
	}
	
}

void Charge_battery(bool end)
{
     
     PORTC |= (1 << PC2);
	 LCD_Goto(4,0);
	 USART_SendStr("Charging\r\n");
	 while(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(CHARGE_TRIGGER_VALUE) && !(PINC & (1 << PC3))){
	  LCD_Goto(4,0);
	  LCD_SendStr("Charging");
	  if(end){
	   printCapacity(Capacity/3600, true, false);
	  }
	  checkBattery(true, false);
	  checkTempPotection();
	 }
     PORTC &= ~(1 << PC2);
	 if(end){
         LCD_Goto(0,0);
	     LCD_SendStr("Full charged! :)");
	     USART_SendStr("Full charged! :)\r\n");
		 printCapacity(Capacity/3600, true, false);
         Reset_Button();
		 USART_SendStr("Rebooting\r\n");
         soft_reset();
	 }
}

void checkEndVoltage(){
	if (END_Voltage > Voltage) { //выключение нагрузки при достижении минимального напряжения
		USART_SendStr("Low voltage: ");
		USART_PutChar((Voltage%10000)/1000);
		USART_PutChar('.');
		USART_PutChar((Voltage%1000)/100);
		USART_PutChar((Voltage%100)/10);
		USART_SendStr("V\r\n");
		OCR1A = 0;
		PORTB &= ~(1 << PB5);
		TIMSK &= ~(1 << OCIE2)|(1 << TOIE2);

		LCD_Clear();
		LCD_Goto(1,0);
		LCD_SendStr("Test completed");
		USART_SendStr("Test completed\r\n");
		
		//Вывод емкости
		printCapacity(Capacity/3600, true, true);

		eeprom_write_dword(&eeLastCapacity, Capacity/3600);

		_delay_ms(1000);

		LCD_Goto(0,0);
		LCD_SendStr("    ");
		LCD_Goto(12,0);
		LCD_SendStr("    ");
		Charge_battery(true);
	}
}

void printVADialig(unsigned long *eeprom, unsigned int step, char *start_text, unsigned int *var, unsigned int position, unsigned int min, unsigned int max, char *uart_text, char *start_value_text, char id){
   LCD_Goto(position, 0);
   LCD_SendStr(start_text);
   *var = eeprom_read_dword(eeprom);
   dialog_id = id;
   USART_SendStr(start_text);
   USART_SendStr("\r\n");
   USART_SendStr(start_value_text);
   USART_PutChar(0x30+(*var/1000));
   USART_PutChar(',');
   USART_PutChar(0x30+((*var%1000)/100));
   USART_SendStr(uart_text);
   while(1){
	 i = BUT_GetBut();
	 button_event = BUT_GetBut();
	 
	 if (USART_GetRxCount()){
		 one_char_buffer = USART_GetChar();
		 PARS_Parser(one_char_buffer);
	 }
	 
	 if(i == PLUS_UP_ID && button_event == BUT_PRESSED_CODE && *var < max){
       *var += step;
	 }

     if(i == MINUS_DOWN_ID && button_event == BUT_PRESSED_CODE && *var > min){
       *var -= step;
	 }
     
	 if((i == ENTER_BUTTON_ID && button_event == BUT_PRESSED_CODE) || value_parsed_success){
	   if(value_parsed_success){
		   *var = int_buffer;
		   value_parsed_success = false;
	   }
       eeprom_write_dword(eeprom, *var);
	   LCD_Clear();
	   break;
	 }

	 LCD_Goto(5, 1);
     LCD_WriteData(1);
	 LCD_WriteData(0x30+(*var/1000));
	 LCD_WriteData(',');
	 LCD_WriteData(0x30+((*var%1000)/100)); 
	 LCD_WriteData(0);
	 BUT_Poll();
   } 
   
}

ISR(TIMER2_OVF_vect)
{
	seconds_timer2++;
	interrupt_data = true;
}

int main()
{
   BUT_Init();
   USART_Init(USART_DOUBLED, 9600);
   PARS_Init();
   LCD_Init();
   t2_init();
   //ADC Init
   ADCSRA = (1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);

   USART_SendStr("Initializing...\r\n");
   
   //Конфиг ножек
   DDRB |= (1 << PB1) | (1 << PB5);
   DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC3) | (1 << PC5));
   DDRC |=  ((1 << PC2) | (1 << PC4));

   LCD_SetUserChar(leftArrow, 0);
   LCD_SetUserChar(rightArrow, 1);


   USART_SendStr("Last capacity:");
   LastCapacity = eeprom_read_dword(&eeLastCapacity);
   LCD_Goto(1,0);
   LCD_SendStr("Last capacity:");
   LCD_Goto(4, 1);
   printCapacity(LastCapacity, false, true);
   LCD_SendStr("mAh");
   USART_SendStr(" mAh\r\n");
   PORTC |= (1 << PC4);
   _delay_ms(200);
   PORTC &=~ (1 << PC4);
   _delay_ms(2500);

   checkBattery(false, false);
   
   //Диалог зарядки 
   if(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(CHARGE_DIALOG_VALUE)){
	 LCD_Clear();
     LCD_Goto(0,0);
     LCD_SendStr("Charge");
	 LCD_Goto(0,1);
	 LCD_SendStr("the battery?");
	 USART_SendStr("Charge the battery? (Y/N)\r\n");
	 while(1){
	     i = BUT_GetBut();
	     button_event = BUT_GetBut();
		 one_char_buffer = USART_GetChar();
		 
		 if((i == PLUS_UP_ID  && button_event == BUT_PRESSED_CODE) || one_char_buffer == 'Y' || one_char_buffer == 'y'){
	       charge_before = true;
		   break;
		 }

	     if(((i == MINUS_DOWN_ID  && button_event == BUT_PRESSED_CODE) || (i == ENTER_BUTTON_ID  && button_event == BUT_PRESSED_CODE)) || one_char_buffer == 'N' || one_char_buffer == 'n'){
		   charge_before = false;
		   break;
		 }
         BUT_Poll();
     }
     LCD_Clear();
   }

  //Установка параметров
  printVADialig(&eeI, AMPERAGE_STEP, "Current:", &I_set, 4, AMPERAGE_MIN, AMPERAGE_MAX, "\r\nSend value 100-2000 with step 100 (or send ok):\r\n", "Default value:", AMPERAGE_DIALOG);
  printVADialig(&eeEND_Voltage, VOLTAGE_STEP, "End voltage:", &END_Voltage, 2, VOLTAGE_MIN, VOLTAGE_MAX, "\r\nSend value 2500-3500 with step 100 (or send ok):\r\n", "Default value:", VOLTAGE_DIALOG);

  //Зарядка перед тестом
  if(charge_before){
     LCD_Clear();
     Charge_battery(false);
  }

   USART_SendStr("Press any key to start the test...\r\n");

   LCD_Goto(0,0);
   LCD_SendStr("Press start to");
   LCD_Goto(0,1);
   LCD_SendStr("begin the test");
   
   
   Reset_Button();


   USART_SendStr("Starting...\r\n");
   LCD_Clear();
   
   USART_SendStr("Seconds | Voltage | Amperage | Time | mAh\r\n");
  
   //ШИМ электронной нагрузки
   TCCR1A |= (1 << COM1A1);
   TCCR1A |= (1 << WGM11) | (1 << WGM10);
   TCCR1B |= (1 << CS11);
   OCR1A = 40*(I_set/100)+4*(I_set/100);
   
   //Подключение АКБ
   PORTB |= (1 << PB5);
   
   //Включение таймера времени
   TIMSK |= (1 << TOIE2);
   
   //Разрешение прерываний
   sei();
   
   while(1)
   {   
	   checkBattery(true, true);
	   //Темапературная  защита
	   checkTempPotection();
	   
       if(interrupt_data) {
		   Voltage = ((read_adc(VOLTAGE_MUX_CHANNEL)*12)/4)*10;
		   I = read_adc(CURRENT_MUX_CHANNEL)*10;

		   //Измерение емкости 
		   Capacity += I;
		   
		   //Вывод секунд
		   char buffer[6];
		   ltoa((long)seconds(), buffer, 10);
		   USART_SendStr(buffer);
		   USART_PutChar(' ');
           
		   
		   //Вывод напряжения 
		   LCD_Goto(0,0);
           printVoltage(Voltage);
		   LCD_SendStr("V");

	       USART_PutChar(' ');

	        //Вывод Тока 
	       LCD_Goto(6,0);
		   printUL(I/1000);
		   LCD_WriteData('.');
		   USART_PutChar('.');
           printUL(I%1000/100);
		   USART_PutChar(0x30+(I%1000)%100/10);
		   LCD_SendStr("A");

	       USART_PutChar(' ');

	       //Вывод времени
		   LCD_Goto(11,0);
	       minutes = seconds()/60;
		   printITime((minutes/60)/10, (minutes/60)%10);
		   USART_PutChar(':');
	       LCD_WriteData(':');
           printITime((minutes%60)/10, (minutes%60%10));
           
	       USART_PutChar(' ');

	       //Вывод емкости
		   LCD_Goto(0,1);
	       printCapacity(Capacity/3600, false, true);
		   LCD_SendStr("mAh");

           /*
	       USARTWriteChar(' ');
    
	       //Вывод Ватт*часов
		   LCD_Goto(9,1);
           printWhVoltage((long)(Wh*1000), true);
		   LCD_SendStr("Wh");
		   */
		   
		   USART_SendStr("\r\n");
		   interrupt_data = false;
      }
	  BUT_Poll();
      i = BUT_GetBut();
      button_event = BUT_GetBut();
      if(i == ENTER_BUTTON_ID && button_event == BUT_PRESSED_CODE){
	   OCR1A = 0;
       //Отключение АКБ
	   PORTB &=~ (1 << PB5);
	   //Выключение таймера времени
	   TIMSK &= ~(1 << TOIE2);
	   USART_SendStr("Test suspended\r\n");

	   LCD_Goto(9, 1);
	   LCD_SendStr("PAUSED");
       i = 0;
	   button_event = 0;
	   while(i != ENTER_BUTTON_ID || button_event != BUT_PRESSED_CODE){
          BUT_Poll();
          i = BUT_GetBut();
          button_event = BUT_GetBut();
	   }
	   USART_SendStr("Initializing...\r\n");
	   //PWM Calc
	   OCR1A = 40*(I_set/100)+4*(I_set/100);
	   
	   //Подключение АКБ
	   PORTB |= (1 << PB5);
	   
	   //Включение таймера времени
	   TIMSK |= (1 << TOIE2);
	   LCD_Goto(9, 1);
	   LCD_SendStr("      ");
	   USART_SendStr("Test continued...\r\n");
      } 

	  //Защита от переразряда и конец теста
	  checkEndVoltage();
   }
}



