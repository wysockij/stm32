#include "stm32f4xx.h"
#include "lcd.h"
#include "main.h"

void lcd_sendHalf(uint8_t data)
{
	LCD_E_HIGH;
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data & 0x01));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (data & 0x02));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (data & 0x04));
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (data & 0x08));
	LCD_E_LOW;
}
void lcd_write_byte(uint8_t data)
{
	lcd_sendHalf(data >> 4);
	lcd_sendHalf(data);
	HAL_Delay(1);
}
void lcd_write_cmd(uint8_t cmd)
{
	LCD_RS_LOW;
	lcd_write_byte(cmd);
}
void lcd_char(char data)
{
	LCD_RS_HIGH;
	lcd_write_byte(data);
}

void lcd_gpio_init(void)
{
	// Włączamy taktowanie portów:
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	// Ustawiamy piny jako wyjścia
	 GPIO_InitTypeDef gpio;

	 gpio.Pin = LCD_E_Pin;
//	 gpio.Pin = LCD_RS_Pin;

	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOB, &gpio);

	 gpio.Pin = LCD_RS_Pin;
	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOB, &gpio);

	 gpio.Pin = LCD_D4_Pin;
//	 gpio.Pin = LCD_D5_Pin;
//	 gpio.Pin = LCD_D6_Pin;
//	 gpio.Pin = LCD_D7_Pin;

	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOE, &gpio);

	 gpio.Pin = LCD_D5_Pin;
	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOE, &gpio);

	 gpio.Pin = LCD_D6_Pin;
	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOE, &gpio);

	 gpio.Pin = LCD_D7_Pin;
	 gpio.Mode = GPIO_MODE_OUTPUT_PP;
	 gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOE, &gpio);

}
void LCD_Init(void)
{
	lcd_gpio_init();

	HAL_Delay(20);

	LCD_E_LOW;
	LCD_RS_LOW;

	lcd_sendHalf(0x03);
	HAL_Delay(5);
	HAL_Delay(1);
	lcd_sendHalf(0x03);
	HAL_Delay(1);
	lcd_sendHalf(0x03);
	HAL_Delay(1);
	lcd_sendHalf(0x02);
	HAL_Delay(1);

	// Już jesteśmy w trybie 4-bitowym. Tutaj dokonujemy ustawień wyświetlacza:
	lcd_write_cmd( LCD_FUNC | LCD_4_BIT | LCDC_TWO_LINE | LCDC_FONT_5x7);
	lcd_write_cmd( LCD_ONOFF | LCD_DISP_ON );
	lcd_write_cmd( LCD_CLEAR );
	HAL_Delay(5);
	lcd_write_cmd( LCDC_ENTRY_MODE | LCD_EM_SHIFT_CURSOR | LCD_EM_RIGHT );

}

void lcd_locate(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			lcd_write_cmd( LCDC_SET_DDRAM | (LCD_LINE1 + x) );
			break;

		case 1:
			lcd_write_cmd( LCDC_SET_DDRAM | (LCD_LINE2 + x) );
			break;
	}
}

void lcd_str(char *text)
{
	while(*text)
		lcd_char(*text++);
}
void lcd_str_XY(uint8_t x, uint8_t y, char *text)
{
	lcd_locate(x,y);

	while(*text)
		lcd_char(*text++);

}
void lcd_clear()
{
	lcd_write_cmd( LCD_CLEAR );
}
