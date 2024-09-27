#include "ssd1306.h"

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static SSD1306_t SSD1306;


//I2C write command
void ssd1306_WriteCommand(uint8_t command)
{
	HAL_I2C_Mem_Write(&hi2c2,SSD1306_I2C_ADDR,0x00,1,&command,1,10);
}


//Initializing SSD1306 chip
uint8_t ssd1306_Init(void)
{	

	HAL_Delay(100);
	
	/* Init LCD */
	ssd1306_WriteCommand(0xAE); //display off
	
	ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode   
	ssd1306_WriteCommand(0x00); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid


	ssd1306_WriteCommand(0x81); //--set contrast control register
	ssd1306_WriteCommand(0xF0);

	ssd1306_WriteCommand(0xA6); //--set normal display
	
	ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_WriteCommand(SSD1306_HEIGHT - 1); //
	
	ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

  ssd1306_WriteCommand(0xD3); //-set display offset
	ssd1306_WriteCommand(0x00); //-not offset

  ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0x80); //--set divide ratio

  ssd1306_WriteCommand(0xD9); //--set pre-charge period
	ssd1306_WriteCommand(0x22); //
	
	ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
	ssd1306_WriteCommand(0x02);	//128*32
	
	ssd1306_WriteCommand(0xDB); //--set vcomh
	ssd1306_WriteCommand(0x40); 
	
	ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	ssd1306_WriteCommand(0x14); //
	ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel
	
	/* Clear screen */
	ssd1306_Fill(Black);
	
	/* Update screen */
	ssd1306_UpdateScreen();
	
	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}


//Fill screen with black or white
void ssd1306_Fill(SSD1306_COLOR color) 
{
	/* Set memory */
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}


//Update RAM to screen
void ssd1306_UpdateScreen(void) 
{
	uint8_t i;
	
	for (i = 0; i < 8; i++) {
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);

		HAL_I2C_Mem_Write(&hi2c2,SSD1306_I2C_ADDR,0x40,1,&SSD1306_Buffer[SSD1306_WIDTH * i],SSD1306_WIDTH,100);
	}
}


/*
Params:
- x: x coordinate 0-127
- y: y coordinate 0-31
- color: Black/White
*/
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
	{
		return;
	}
	
	if (SSD1306.Inverted) 
	{
		color = (SSD1306_COLOR)!color;
	}
	
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} 
	else 
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}



/*
Writing single character function
Params:
- ch: character to be displayed
- Font: font selection Font_7x10/Font_11x18/Font_16x26
- color: Black/White
*/
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	

	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{

		return 0;
	}
	

	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	

	SSD1306.CurrentX += Font.FontWidth;
	
	return ch;
}

/*
Writing string function
Params:
- str: pointer to string
- Font: font selection Font_7x10/Font_11x18/Font_16x26
- color: Black/White
*/
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{

	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			
			return *str;
		}
		
		str++;
	}
	
	return *str;
}


//Set cursor coordinate
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}
