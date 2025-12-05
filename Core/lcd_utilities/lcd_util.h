#ifndef LCD_UTILITIES
#define LCD_UTILITIES

#include "lcd.h"
#include <stdio.h>

void LCD_PrintString16(uint16_t Xpos, uint16_t Ypos, char* chars, uint16_t sz);

void LCD_PrintUnsigned32Hex(uint16_t Xpos, uint16_t Ypos, uint32_t n);

#endif
