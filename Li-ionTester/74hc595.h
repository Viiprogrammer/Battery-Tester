#ifndef SHIFTREG_H
#define SHIFTREG_H
 void ShiftRegisterInit(void);
 void ShiftRegisterSend(void);
 void ShiftDigitalWrite(int pin, int lvl, int number);
 void ShiftDigitalWritePort(int port, int number);
 char ShiftDigitalGetPort(int number);
 //#define USE_HARDWARE_SPI
 
 #define DATA 0
 #define SCK 3
 #define LATCH 4

 #define DATA_DDR DDRB
 #define SCK_DDR DDRB
 #define LATCH_DDR DDRB

 #define DATA_PORT PORTB
 #define SCK_PORT PORTB
 #define LATCH_PORT PORTB
 
#endif //SHIFTREG_H