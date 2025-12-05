#include "lcd_util.h"

void LCD_PrintString16(uint16_t Xpos, uint16_t Ypos, char* chars, uint16_t sz) {
	for (uint16_t i = 0; i < sz; i++) {
		LCD_DisplayChar(Xpos, Ypos - 16*i, chars[i]);
	}
}


void LCD_PrintUnsigned32Hex(uint16_t Xpos, uint16_t Ypos, uint32_t n) {
	char buf[12];
	uint8_t p = 0;
	uint8_t start_zero = 1;
	snprintf(buf, sizeof(buf), "%08x", n);
	for (int i = 0; i < 8; i++) {
		if (buf[i] == '0' && start_zero) continue;
		LCD_DisplayChar(Xpos, Ypos - 16*p, buf[i]);
		start_zero = 0;
		p++;
	}
}
