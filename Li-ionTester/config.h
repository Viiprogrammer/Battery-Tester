#ifndef CONFIG_H
#define CONFIG_H

	#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR)) //Установка внешней опорки
	#define V_REF 2.56 //Опорка
	#define ADC_READ_NUM 3
	#define MINUS_DOWN_ID 3
	#define CALC_ADC_VOLTAGE(y) (int)(y/(V_REF/1023))
	#define NO_BATTERY_VALUE 0.1 //Все что ниже не считает за наличие акб
	#define CHARGE_TRIGGER_VALUE 4 //Значиние при котором считать акб заряженым (доп проверка кроме светодиода TP4056) 4V - 899
	#define CHARGE_DIALOG_VALUE 4.05 //Ниже какогого напряжения выводить предложение зарядки?
	#define BATTERY_CRITICAL_TEMP_VALUE 851
	#define CURRENT_MUX_CHANNEL 1
	#define VOLTAGE_MUX_CHANNEL 0
	#define VOLTAGE_DIALOG 1
	#define AMPERAGE_DIALOG 2
	
	#define VOLTAGE_MAX 3500
	#define VOLTAGE_MIN 2500
	#define VOLTAGE_STEP 100
	
	#define AMPERAGE_MAX 2000
	#define AMPERAGE_MIN 100
	#define AMPERAGE_STEP 100
	
	#define ENTER_BUTTON_ID 1
	#define PLUS_UP_ID 2
	#define MINUS_DOWN_ID 3
	
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <util/delay.h>
	#include <avr/eeprom.h>
	#include <avr/sleep.h>
	#include <avr/wdt.h>
	#include <stdlib.h>
	#include <stdbool.h>
#endif //CONFIG_H