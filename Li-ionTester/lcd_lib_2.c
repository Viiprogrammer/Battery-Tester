#include "lcd_lib_2.h"
#include "config.h"
/*������� ��� ������ � ������ � ������
��� ������� �� ����� ����� ��������������*/

/*________________________________________________________________*/
          
#define LCD_COM_ENTRY_MODE_SET     (1<<2)|(LCD_DEC_INC_DDRAM<<1)|(LCD_SHIFT_RIGHT_LEFT<<0)
#define LCD_COM_DISPLAY_CONTR      (1<<3)|(LCD_DISPLAY_OFF_ON<<2)|(LCD_CURSOR_OFF_ON<<1)|(LCD_CURSOR_BLINK_OFF_ON<<0)
#define LCD_COM_FUNCTION_SET       (1<<5)|(LCD_BUS_4_8_BIT<<4)|(LCD_ONE_TWO_LINE<<3)|(LCD_FONT58_FONT511<<2)
#define LCD_COM_INIT_1              0x30
#define LCD_DELAY_STROB             2
#define LCD_DELAY_WAIT              40
#define LCD_FL_BF                   7

/*_________________________________________________________________*/

void LCD_WriteComInit(uint8_t data)
{
  delay_us(LCD_DELAY_WAIT);
  ShiftDigitalWrite(1, LOW, 0); 
  
#if (LCD_BUS_4_8_BIT == 0)
  data &= 0xf0;
#endif
  
  ShiftDigitalWritePort(((ShiftDigitalGetPort(0) & 0b11000011) | (data >> 2)), 0);
  ShiftDigitalWrite(0, HIGH, 0);
  delay_us(LCD_DELAY_STROB);
  ShiftDigitalWrite(0, LOW, 0);
}


/*����� �������*/
inline static void LCD_CommonFunc(uint8_t data)
{
#if (LCD_BUS_4_8_BIT == 0) 
  
  uint8_t tmp; 
  tmp = (data & 0xf0);
  ShiftDigitalWritePort(((ShiftDigitalGetPort(0) & 0b11000011) | (tmp >> 2)), 0);
  ShiftDigitalWrite(0, HIGH, 0);
  delay_us(LCD_DELAY_STROB);
  ShiftDigitalWrite(0, LOW, 0);

  data = __swap_nibbles(data); 
  tmp = (data & 0xf0);
    
  ShiftDigitalWritePort(((ShiftDigitalGetPort(0) & 0b11000011) | (tmp >> 2)), 0);
  ShiftDigitalWrite(0, HIGH, 0);
  delay_us(LCD_DELAY_STROB);
  ShiftDigitalWrite(0, LOW, 0);
  
#else 
  
  LCD_WritePort(LCD_PORT, data);
  ShiftDigitalWrite(0, HIGH, 0);
  delay_us(LCD_DELAY_STROB);
  ShiftDigitalWrite(0, LOW, 0);
  
#endif
}

/*������� �������� ���������� lcd*/
INLINE static void LCD_Wait(void)
{
#if (LCD_CHECK_FL_BF == 1)
 #if (LCD_BUS_4_8_BIT == 0)
  
  uint8_t data, tmp;
  LCD_DirPort(LCD_PORT, 0x00);
  LCD_WritePort(LCD_PORT, 0xff);
  LCD_SetPin(LCD_RW);
  ShiftDigitalWrite(1, LOW, 0);
  do{
    ShiftDigitalWrite(0, HIGH, 0);
    delay_us(LCD_DELAY_STROB);
    LCD_ReadPort(LCD_PORT, data);
    ShiftDigitalWrite(0, LOW, 0);
   
    ShiftDigitalWrite(0, HIGH, 0);
    delay_us(LCD_DELAY_STROB);
    LCD_ReadPort(LCD_PORT, tmp);
    ShiftDigitalWrite(0, LOW, 0); 
    
  } while((data & (1<<LCD_FL_BF))!= 0);
  LCD_ClearPin(LCD_RW);
  LCD_DirPort(LCD_PORT, 0xff);
  
  #else
  
  uint8_t data;
  LCD_DirPort(LCD_PORT, 0x00);
  LCD_WritePort(LCD_PORT, 0xff);
  LCD_SetPin(LCD_RW);
  ShiftDigitalWrite(1, LOW, 0);
  do{
     ShiftDigitalWrite(0, HIGH, 0);
     delay_us(LCD_DELAY_STROB);
     LCD_ReadPort(LCD_PORT, data);
     ShiftDigitalWrite(0, LOW, 0);
  } while((data & (1<<LCD_FL_BF))!= 0);
  LCD_ClearPin(LCD_RW);
  LCD_DirPort(LCD_PORT, 0xff);
   
  #endif    
#else
  delay_us(LCD_DELAY_WAIT);
#endif  
}

/*������� ������ �������*/
void LCD_WriteCom(uint8_t data)
{
  LCD_Wait();
  ShiftDigitalWrite(1, LOW, 0);
  LCD_CommonFunc(data);
}

/*������� ������ ������*/
void LCD_WriteData(char data)
{
  LCD_Wait();
  ShiftDigitalWrite(1, HIGH, 0);    
  LCD_CommonFunc(data);
}

/*������� �������������*/
void LCD_Init(void)
{  
  LCD_WriteComInit(LCD_COM_INIT_1); 
  delay_ms(10);
  LCD_WriteComInit(LCD_COM_INIT_1);
  delay_ms(2);
  LCD_WriteComInit(LCD_COM_INIT_1);
  
#if (LCD_BUS_4_8_BIT == 0) 
  LCD_WriteComInit(LCD_COM_FUNCTION_SET);
#endif

  LCD_WriteCom(LCD_COM_FUNCTION_SET);
  LCD_WriteCom(LCD_COM_DISPLAY_CONTR);  
  LCD_WriteCom(LCD_CLEAR_DISPLAY);  
  delay_ms(2);
  LCD_WriteCom(LCD_COM_ENTRY_MODE_SET); 
  
}

/*������� ����� ������ �� ���*/
void LCD_SendStr(char *str)
{
  uint8_t data;
  while (*str){
    data =  *str++;
    LCD_WriteData(data);
  }
}


#ifdef __GNUC__

/*������� ������ ������ �� ���� ������*/
void LCD_SendStrFl(char *str)
{
  char data;			
  while (*str){
    data = pgm_read_byte(str);
    str++;
    LCD_WriteData(data);
  }
}

void LCD_SetUserChar(uint8_t const *sym, uint8_t adr)
{
   uint8_t data;	
   uint8_t i;

   LCD_WriteCom((1<<0x06)|((adr&0x07)<<0x03));
   
   i = 0;
   while (i<8){
      data = pgm_read_byte(sym);
      sym++;
      LCD_WriteData(data);
      i++;
   }    
}

//��� IARa � CodeVision
#else

/*������� ������ ������ �� ���� ������*/
void LCD_SendStrFl(char __flash *str)
{
  char data;			
  while (*str){
    data = *str++;
    LCD_WriteData(data);
  }
}

void LCD_SetUserChar(uint8_t __flash *sym, uint8_t adr)
{
   uint8_t data;	
   uint8_t i;

   LCD_WriteCom((1<<0x06)|((adr&0x07)<<0x03));
   
   i = 0;
   while (i<8){
      data = *sym++;
      LCD_WriteData(data);
      i++;
   }    
}

#endif