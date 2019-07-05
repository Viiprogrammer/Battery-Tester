#ifndef CONFIG_H
#define CONFIG_H

	#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR)) //��������� ������� ������
	#define V_REF 4.55 //������
	#define CALC_ADC_VOLTAGE(y) (int)(y/(V_REF/1023))
	#define NO_BATTERY_VALUE 0.1 //��� ��� ���� �� ������� �� ������� ���
	#define CHARGE_TRIGGER_VALUE 4 //�������� ��� ������� ������� ��� ��������� (��� �������� ����� ���������� TP4056) 4V - 899
	#define CHARGE_DIALOG_VALUE 4.05 //���� �������� ���������� �������� ����������� �������?
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
	
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <util/delay.h>
	#include <avr/eeprom.h>
	#include <avr/sleep.h>
	#include <avr/wdt.h>
	#include <stdlib.h>
	#include <stdbool.h>
#endif //CONFIG_H