#include "config.h"
#include "74hc595.h"
char ShiftPORT[] = {0b00000000};
void ShiftRegisterInit(){
  	DATA_DDR |= (1<<DATA);
	SCK_DDR |= (1<<SCK);
	LATCH_DDR |= (1<<LATCH);
	
  	DATA_PORT &=~ (1 << DATA);
	SCK_PORT &=~ (1 << SCK);
	LATCH_PORT &=~ (1 << LATCH);
	#ifdef USE_HARDWARE_SPI
	 SPCR = ((1<<SPE)|(1<<MSTR));//Включение, режим MASTER
	#endif
}
void ShiftRegisterSend(){
	unsigned int inc_data = 0;
	#ifdef USE_HARDWARE_SPI
	for(inc_data = 0; inc_data <= sizeof(ShiftPORT); inc_data++){
		SPDR = ShiftPORT[inc_data];
		while(!(SPSR & (1<<SPIF)));
	}
	#endif
	#ifndef USE_HARDWARE_SPI
	for(inc_data = 0; inc_data <= sizeof(ShiftPORT); inc_data++){
		unsigned char data = ShiftPORT[inc_data];
		for(inc_data = 0; inc_data < 8; inc_data++){
			if(data & 0x80){
				DATA_PORT |= (1 << DATA);
			}else{
				DATA_PORT &=~ (1 << DATA);
			}
			SCK_PORT |= (1 << SCK);
			SCK_PORT &=~ (1 << SCK);
			data <<= 1;
		}
	}
	#endif   
	LATCH_PORT |= (1 << LATCH);
	LATCH_PORT &=~ (1 << LATCH);
}
void ShiftDigitalWrite(int pin, int lvl, int number){
  if(lvl){
    ShiftPORT[number] |= (1 << pin);
  }else{
	ShiftPORT[number] &= ~(1 << pin);
  }
  ShiftRegisterSend();
}
void ShiftDigitalWritePort(int port, int number){
    ShiftPORT[number] = port;
	ShiftRegisterSend();
}
char ShiftDigitalGetPort(int number){
	 return ShiftPORT[number];
}