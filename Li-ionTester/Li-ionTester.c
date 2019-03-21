/*
  Реализовать выбор напряжения оконьчания теста
  TP4056 зарядка, отслеживание зарядки по 2ум светодиодам 
   - пауза теста 
   - индикация нажатия кнопок если работает то зеленый если нет то красный (есть звуковая)
   - хардвар защита по температуре (есть софтвар)
   - Добавить защиту от переполюсовки, и индикацию неверного подключения
   - Прицепить энкодер
*/
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR)) //Установка внешней опорки
#define V_REF 4.55 //Опорка
#define CALC_ADC_VOLTAGE(y) (int)(y/(V_REF/1023))
#define NO_BATTERY_VALUE 0.1 //Все что ниже не считает за наличие акб
#define CHARGE_TRIGGER_VALUE 4 //Значиние при котором считать акб заряженым (доп проверка кроме светодиода TP4056) 4V - 899
#define CHARGE_DIALOG_VALUE 4.05 //Ниже какогого напряжения выводить предложение зарядки?
#define BATTERY_CRITICAL_TEMP_VALUE 851
#define CURRENT_MUX_CHANNEL 1
#define VOLTAGE_MUX_CHANNEL 0
#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <stdbool.h>
#include "buttons.h"

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


void t2_init(){
   TIMSK &= ~(1 << OCIE2)|(1 << TOIE2);
   ASSR |= (1 << AS2);
   TCNT2 = 0;
   TCCR2 |= (1 << CS22)|(1 << CS20);
}

void USARTInit(uint16_t ubrr_value)
{
   UBRRL = ubrr_value;
   UBRRH = (ubrr_value>>8);
   UCSRC=(1<<URSEL)|(3<<UCSZ0);
   UCSRB=(1<<RXEN)|(1<<TXEN);
}

char USARTReadChar()
{
   while(!(UCSRA & (1<<RXC))){}
   return UDR;
}

void USARTWriteChar(char data)
{
   while(!(UCSRA & (1<<UDRE))){}
   UDR = data;
}
void USARTWrite(char *str)
{
  uint8_t data;
  while (*str){
    data =  *str++;
    USARTWriteChar(data);
  }
}

unsigned int read_adc(unsigned char adc_input)
{
  ADMUX= adc_input | ADC_VREF_TYPE;
  _delay_us(10);
  ADCSRA|=(1<<ADSC);
  while ((ADCSRA & (1<<ADIF))==0);
  ADCSRA|=(1<<ADIF);
  return ADCW;
}

/*
  Подзалупные функции для экономии памяти 
*/
void printUL(char i)
{
    LCD_WriteData(0x30+i);
    USARTWriteChar(0x30+i);
}
void printUARTLCD(char i, bool uart)
{
	LCD_WriteData(0x30+i);
	if(uart){
		USARTWriteChar(0x30+i);
	}
}
void printITime(char a, char b)
{
    LCD_WriteData(0x30+a);
	LCD_WriteData(0x30+b);
    USARTWriteChar(0x30+a);
    USARTWriteChar(0x30+b);
}

void printWhVoltage(unsigned long val, bool wh)
{
   if(wh){
     printUL(val/10000);
   }
   printUL((val%10000)/1000);
   LCD_WriteData('.');
   USARTWriteChar('.');
   printUL((val%1000)/100);
   printUL((val%100)/10);
}
/* Конец долбоебизма */

void checkBattery()
{
   if(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(NO_BATTERY_VALUE)){
     LCD_Goto(0,0);
     LCD_SendStr("Please connect");
	 LCD_Goto(0,1);
	 LCD_SendStr("the battery ");
	 while(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(NO_BATTERY_VALUE)){}
     LCD_Clear();
   }
}

void Reset_Button(){
	while(BUT_GetKey() != 1){
		if((UCSRA & (1<<RXC))) break;
		BUT_Debrief();
	}
}

void checkTempPotection(){
	if(read_adc(5) >= BATTERY_CRITICAL_TEMP_VALUE){
		 LCD_Clear();
		 LCD_Goto(0,0);
		 LCD_SendStr("High temperature");
		 LCD_Goto(1,1);
		 LCD_SendStr("Enter - reboot");
		 USARTWrite("Critical temperarure!!! Test Stopped\r\n");
	     cli();
	     OCR1A = 0;
         PORTB &= ~(1 << PB5);
         TCCR1B &= ~(1 << CS11);	
         Reset_Button();
         USARTWrite("Rebooting\r\n");
         soft_reset();
    }
}

void printCapacity(unsigned long Capacity, bool mode, bool uart){
	if(mode){
		LCD_Goto(4,1);
		if(uart){
			USARTWrite("Capacity: ");
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
			USARTWrite(" mAh\r\n");
		}
		LCD_SendStr("mAh");
	}
	
}

void Charge_battery(bool end)
{
     
     PORTC |= (1 << PC2);
	 LCD_Goto(4,0);
	 USARTWrite("Charging\r\n");
	 while(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(CHARGE_TRIGGER_VALUE) && !(PINC & (1 << PC3))){
	  LCD_Goto(4,0);
	  LCD_SendStr("Charging");
	  if(end){
	   printCapacity(Capacity/3600, true, false);
	  }
	  checkBattery();
	  checkTempPotection();
	 }
     PORTC &= ~(1 << PC2);
	 if(end){
         LCD_Goto(0,0);
	     LCD_SendStr("Full charged! :)");
	     USARTWrite("Full charged! :)\r\n");
		 printCapacity(Capacity/3600, true, false);
         Reset_Button();
		 USARTWrite("Rebooting\r\n");
         soft_reset();
	 }
}

void printVADialig(unsigned long *eeprom, unsigned int step, char *start_text, unsigned int *var, unsigned int position, unsigned int min, unsigned int max){
   LCD_Goto(position, 0);
   LCD_SendStr(start_text);
   *var = eeprom_read_dword(eeprom);
   USARTWrite("Press any key to skip selection\r\n");
   while(1){
	 i = BUT_GetKey();
	 if(i == 3 && *var < max){
       *var += step;
	 }

     if(i == 4 && *var > min){
       *var -= step;
	 }

	 if(i == 1 || (UCSRA & (1<<RXC))){
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
	 BUT_Debrief();
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
   USARTInit(51);
   LCD_Init();
   t2_init();
   //ADC Init
   ADCSRA = (1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);

   USARTWrite("Initializing...\r\n");
   
   //Конфиг ножек
   DDRB |= (1 << PB1) | (1 << PB5);
   DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC3) | (1 << PC5));
   DDRC |=  ((1 << PC2) | (1 << PC4));

   LCD_SetUserChar(leftArrow, 0);
   LCD_SetUserChar(rightArrow, 1);


   USARTWrite("Last capacity:");
   LastCapacity = eeprom_read_dword(&eeLastCapacity);
   LCD_Goto(1,0);
   LCD_SendStr("Last capacity:");
   LCD_Goto(4, 1);
   printCapacity(LastCapacity, false, true);
   LCD_SendStr("mAh");
   USARTWrite(" mAh\r\n");
   PORTC |= (1 << PC4);
   _delay_ms(200);
   PORTC &=~ (1 << PC4);
   _delay_ms(2500);

   LCD_Clear();

   checkBattery();
   
   //Диалог зарядки 
   if(read_adc(VOLTAGE_MUX_CHANNEL) < CALC_ADC_VOLTAGE(CHARGE_DIALOG_VALUE)){
     LCD_Goto(0,0);
     LCD_SendStr("Charge");
	 LCD_Goto(0,1);
	 LCD_SendStr("the battery?");
	 USARTWrite("Press any key to skip selection\r\n");
	 while(1){
	     i = BUT_GetKey();
		 if(i == 3){
	       charge_before = true;
		   break;
		 }

	     if(i == 4 || i == 1 || (UCSRA & (1<<RXC))){
		   charge_before = false;
		   break;
		 }
         BUT_Debrief();
     }
     LCD_Clear();
   }

  //Установка параметров
  printVADialig(&eeI, 100, "Current:", &I_set, 4, 100, 2000);
  printVADialig(&eeEND_Voltage, 100, "End voltage:", &END_Voltage, 2, 2500, 3500);

  //Зарядка перед тестом
  if(charge_before){
     LCD_Clear();
     Charge_battery(false);
  }

   USARTWrite("Press any key to start the test...\r\n");

   LCD_Goto(0,0);
   LCD_SendStr("Press start to");
   LCD_Goto(0,1);
   LCD_SendStr("begin the test");
   
   
   Reset_Button();


   USARTWrite("Starting...\r\n");
   LCD_Clear();
   
   USARTWrite("Seconds | Voltage | Amperage | Time | mAh | Wh\r\n");
  
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
	   checkTempPotection();

       if(interrupt_data) {
		   Voltage = (read_adc(VOLTAGE_MUX_CHANNEL)*455000/1023000)*10;
		   I = read_adc(CURRENT_MUX_CHANNEL)*10;

		   //Измерение емкости 
		   Capacity += I;
		   
		   //Вывод секунд
		   char buffer[6];
		   ltoa((long)seconds(), buffer, 10);
		   USARTWrite(buffer);
		   USARTWriteChar(' ');
           
		   
		   //Вывод напряжения 
		   LCD_Goto(0,0);
           printWhVoltage(Voltage, false);
		   LCD_SendStr("V");

	       USARTWriteChar(' ');

	        //Вывод Тока 
	       LCD_Goto(6,0);
		   printUL(I/1000);
		   LCD_WriteData('.');
		   USARTWriteChar('.');
           printUL(I%1000/100);
		   USARTWriteChar(0x30+(I%1000)%100/10);
		   LCD_SendStr("A");

	       USARTWriteChar(' ');

	       //Вывод времени
		   LCD_Goto(11,0);
	       minutes = seconds()/60;
		   printITime((minutes/60)/10, (minutes/60)%10);
		   USARTWriteChar(':');
	       LCD_WriteData(':');
           printITime((minutes%60)/10, (minutes%60%10));
           
	       USARTWriteChar(' ');

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
		   
		   USARTWrite("\r\n");
		   interrupt_data = false;
      }
      
	  if (END_Voltage > Voltage) { //выключение нагрузки при достижении минимального напряжения
	     cli();
	     OCR1A = 0;
         PORTB &= ~(1 << PB5);
         TIMSK &= ~(1 << OCIE2)|(1 << TOIE2);

		 LCD_Clear();
		 LCD_Goto(1,0);
		 LCD_SendStr("Test completed");
         USARTWrite("Test completed\r\n");
		 
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
}



