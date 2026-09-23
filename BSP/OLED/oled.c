#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

uint8_t OLED_GRAM[144][8];

// Invert screen color display
void OLED_ColorTurn(uint8_t i)
{
	if(i == 0)
	{
		OLED_WR_Byte(0xA6, OLED_CMD); // Normal display
	}
	if(i == 1)
	{
		OLED_WR_Byte(0xA7, OLED_CMD); // Inverted display
	}
}

// Rotate screen display by 180 degrees
void OLED_DisplayTurn(uint8_t i)
{
	if(i == 0)
	{
		OLED_WR_Byte(0xC8, OLED_CMD); // Normal COM scan
		OLED_WR_Byte(0xA1, OLED_CMD);
	}
	if(i == 1)
	{
		OLED_WR_Byte(0xC0, OLED_CMD); // Flipped COM scan
		OLED_WR_Byte(0xA0, OLED_CMD);
	}
}

// Tiny delay for software I2C timing matching
void IIC_delay(void)
{
	uint8_t t = 3;
	while(t--);
}

// I2C start signal
void I2C_Start(void)
{
	OLED_SDA_Set();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SDA_Clr();
	IIC_delay();
	OLED_SCL_Clr();
	IIC_delay();
}

// I2C stop signal
void I2C_Stop(void)
{
	OLED_SDA_Clr();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SDA_Set();
}

// Wait for I2C slave ACK response
void I2C_WaitAck(void)
{
	OLED_SDA_Set();
	IIC_delay();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SCL_Clr();
	IIC_delay();
}

// Send one byte via software I2C
void Send_Byte(uint8_t dat)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if(dat & 0x80) // Send MSB first
		{
			OLED_SDA_Set();
		}
		else
		{
			OLED_SDA_Clr();
		}
		IIC_delay();
		OLED_SCL_Set();
		IIC_delay();
		OLED_SCL_Clr(); // Clock falling edge latch data
		dat <<= 1;
	}
}

// Write 1 byte to OLED
// mode: 0=command, 1=data
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
	I2C_Start();
	Send_Byte(0x7A);
	I2C_WaitAck();
	if(mode)
	{
		Send_Byte(0x40);
	}
	else
	{
		Send_Byte(0x00);
	}
	I2C_WaitAck();
	Send_Byte(dat);
	I2C_WaitAck();
	I2C_Stop();
}

// Turn OLED display ON
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD); // Charge pump setting
	OLED_WR_Byte(0x14, OLED_CMD); // Enable charge pump
	OLED_WR_Byte(0xAF, OLED_CMD); // Screen power on
}

// Turn OLED display OFF
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD); // Charge pump setting
	OLED_WR_Byte(0x10, OLED_CMD); // Disable charge pump
	OLED_WR_Byte(0xAE, OLED_CMD); // Screen power off
}

// Refresh full OLED screen from local GRAM buffer
void OLED_Refresh(void)
{
	uint8_t i, n;
	for(i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD); // Set page start address
		OLED_WR_Byte(0x00, OLED_CMD);     // Low column address
		OLED_WR_Byte(0x10, OLED_CMD);     // High column address
		I2C_Start();
		Send_Byte(0x7A);
		I2C_WaitAck();
		Send_Byte(0x40);
		I2C_WaitAck();
		for(n = 0; n < 128; n++)
		{
			Send_Byte(OLED_GRAM[n][i]);
			I2C_WaitAck();
		}
		I2C_Stop();
	}
}

// Clear entire OLED local GRAM buffer and refresh screen
void OLED_Clear(void)
{
	uint8_t i, n;
	for(i = 0; i < 8; i++)
	{
		for(n = 0; n < 128; n++)
		{
			OLED_GRAM[n][i] = 0; // Fill all pixels black
		}
	}
	OLED_Refresh(); // Update physical screen
}

// Draw single pixel
// x:0~127, y:0~63
// t=1 light pixel, t=0 dark pixel
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
	uint8_t i, m, n;
	i = y / 8;
	m = y % 8;
	n = 1 << m;
	if(t)
	{
		OLED_GRAM[x][i] |= n;
	}
	else
	{
		OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
		OLED_GRAM[x][i] |= n;
		OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
	}
}

// Draw straight line using Bresenham algorithm
// (x1,y1) start point, (x2,y2) end point
// mode: pixel on/off
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
	uint16_t t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; // Total X axis offset
	delta_y = y2 - y1;
	uRow = x1; // Current X coordinate
	uCol = y1; // Current Y coordinate

	if(delta_x > 0)
		incx = 1;
	else if(delta_x == 0)
		incx = 0; // Vertical line
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}

	if(delta_y > 0)
		incy = 1;
	else if(delta_y == 0)
		incy = 0; // Horizontal line
	else
	{
		incy = -1;
		delta_y = -delta_y;
	}

	if(delta_x > delta_y)
		distance = delta_x; // Use longer axis as step count
	else
		distance = delta_y;

	for(t = 0; t < distance + 1; t++)
	{
		OLED_DrawPoint(uRow, uCol, mode);
		xerr += delta_x;
		yerr += delta_y;
		if(xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if(yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

// Draw circle
// x,y: circle center coordinate
// r: circle radius
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
	int a, b, num;
	a = 0;
	b = r;
	while(2 * b * b >= r * r)
	{
		OLED_DrawPoint(x + a, y - b, 1);
		OLED_DrawPoint(x - a, y - b, 1);
		OLED_DrawPoint(x - a, y + b, 1);
		OLED_DrawPoint(x + a, y + b, 1);

		OLED_DrawPoint(x + b, y + a, 1);
		OLED_DrawPoint(x + b, y - a, 1);
		OLED_DrawPoint(x - b, y - a, 1);
		OLED_DrawPoint(x - b, y + a, 1);

		a++;
		num = (a * a + b * b) - r * r; // Judge distance error from standard circle
		if(num > 0)
		{
			b--;
			a--;
		}
	}
}

// Display single ASCII character
// x,y: display start coordinate
// size1: font size options: 8/12/16/24
// mode:0=black font on white bg, 1=white font on black bg
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
	uint8_t i, m, temp, size2, chr1;
	uint8_t x0 = x, y0 = y;
	if(size1 == 8)
		size2 = 6;
	else
		size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
	chr1 = chr - ' '; // Offset ASCII to font table index

	for(i = 0; i < size2; i++)
	{
		if(size1 == 8)
			temp = asc2_0806[chr1][i];
		else if(size1 == 12)
			temp = asc2_1206[chr1][i];
		else if(size1 == 16)
			temp = asc2_1608[chr1][i];
		else if(size1 == 24)
			temp = asc2_2412[chr1][i];
		else
			return;

		for(m = 0; m < 8; m++)
		{
			if(temp & 0x01)
				OLED_DrawPoint(x, y, mode);
			else
				OLED_DrawPoint(x, y, !mode);
			temp >>= 1;
			y++;
		}
		x++;
		if((size1 != 8) && ((x - x0) == size1 / 2))
		{
			x = x0;
			y0 = y0 + 8;
		}
		y = y0;
	}
}

// Display ASCII string
// x,y: start coordinate
// size1: font size
// *chr: pointer to string buffer
// mode: display inversion mode
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
	while((*chr >= ' ') && (*chr <= '~')) // Only printable ASCII
	{
		OLED_ShowChar(x, y, *chr, size1, mode);
		if(size1 == 8)
			x += 6;
		else
			x += size1 / 2;
		chr++;
	}
}

// Calculate m^n
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while(n--)
	{
		result *= m;
	}
	return result;
}

// Display decimal number
// x,y: coordinate
// num: number to print
// len: total digit length
// size1: font size
// mode: color inversion
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
	uint8_t t, temp, m = 0;
	if(size1 == 8)
		m = 2;
	for(t = 0; t < len; t++)
	{
		temp = (num / OLED_Pow(10, len - t - 1)) % 10;
		if(temp == 0)
		{
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
		}
		else
		{
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
		}
	}
}

// Display Chinese character
// num: index of chinese font library
// size1: font size 16/24/32/64
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
	uint8_t m, temp;
	uint8_t x0 = x, y0 = y;
	uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;

	for(i = 0; i < size3; i++)
	{
		if(size1 == 16)
			temp = Hzk1[num][i];
		else if(size1 == 24)
			temp = Hzk2[num][i];
		else if(size1 == 32)
			temp = Hzk3[num][i];
		else if(size1 == 64)
			temp = Hzk4[num][i];
		else
			return;

		for(m = 0; m < 8; m++)
		{
			if(temp & 0x01)
				OLED_DrawPoint(x, y, mode);
			else
				OLED_DrawPoint(x, y, !mode);
			temp >>= 1;
			y++;
		}
		x++;
		if((x - x0) == size1)
		{
			x = x0;
			y0 = y0 + 8;
		}
		y = y0;
	}
}

// Horizontal scrolling Chinese text display
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
	uint8_t i, n, t = 0, m = 0, r;
	while(1)
	{
		if(m == 0)
		{
			OLED_ShowChinese(128, 24, t, 16, mode);
			t++;
		}
		if(t == num)
		{
			for(r = 0; r < 16 * space; r++)
			{
				for(i = 1; i < 144; i++)
				{
					for(n = 0; n < 8; n++)
					{
						OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
					}
				}
				OLED_Refresh();
			}
			t = 0;
		}
		m++;
		if(m == 16)
		{
			m = 0;
		}
		for(i = 1; i < 144; i++) // Move all pixels left by one column
		{
			for(n = 0; n < 8; n++)
			{
				OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
	}
}

// Show monochrome bitmap picture
// sizex/sizey: picture width/height
// BMP[]: bitmap data array
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
	uint16_t j = 0;
	uint8_t i, n, temp, m;
	uint8_t x0 = x, y0 = y;
	sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
	for(n = 0; n < sizey; n++)
	{
		for(i = 0; i < sizex; i++)
		{
			temp = BMP[j];
			j++;
			for(m = 0; m < 8; m++)
			{
				if(temp & 0x01)
					OLED_DrawPoint(x, y, mode);
				else
					OLED_DrawPoint(x, y, !mode);
				temp >>= 1;
				y++;
			}
			x++;
			if((x - x0) == sizex)
			{
				x = x0;
				y0 = y0 + 8;
			}
			y = y0;
		}
	}
}

// OLED hardware initialization sequence
void OLED_Init(void)
{
	// All GPIO pins are initialized by CubeMX MX_GPIO_Init() in main.c
	// Delete all standard library GPIO init code here

//	OLED_RES_Clr();
//	HAL_Delay(200);
//	OLED_RES_Set();

	OLED_WR_Byte(0xAE, OLED_CMD); // Turn OLED display off
	OLED_WR_Byte(0x00, OLED_CMD); // Set low column address
	OLED_WR_Byte(0x10, OLED_CMD); // Set high column address
	OLED_WR_Byte(0x40, OLED_CMD); // Set display RAM start line
	OLED_WR_Byte(0x81, OLED_CMD); // Contrast adjustment
	OLED_WR_Byte(0xCF, OLED_CMD); // Brightness setting
	OLED_WR_Byte(0xA1, OLED_CMD); // Column segment remap
	OLED_WR_Byte(0xC8, OLED_CMD); // COM scan direction remap
	OLED_WR_Byte(0xA6, OLED_CMD); // Normal display mode
	OLED_WR_Byte(0xA8, OLED_CMD); // Multiplex ratio setting
	OLED_WR_Byte(0x3f, OLED_CMD); // 1/64 duty cycle
	OLED_WR_Byte(0xD3, OLED_CMD); // Display vertical offset
	OLED_WR_Byte(0x00, OLED_CMD); // No offset
	OLED_WR_Byte(0xd5, OLED_CMD); // Clock frequency & divider
	OLED_WR_Byte(0x80, OLED_CMD); // Oscillator frequency
	OLED_WR_Byte(0xD9, OLED_CMD); // Pre-charge timing
	OLED_WR_Byte(0xF1, OLED_CMD);
	OLED_WR_Byte(0xDA, OLED_CMD); // COM pin hardware config
	OLED_WR_Byte(0x12, OLED_CMD);
	OLED_WR_Byte(0xDB, OLED_CMD); // VCOM deselect voltage
	OLED_WR_Byte(0x40, OLED_CMD);
	OLED_WR_Byte(0x20, OLED_CMD); // Page addressing mode
	OLED_WR_Byte(0x02, OLED_CMD);
	OLED_WR_Byte(0x8D, OLED_CMD); // Charge pump control
	OLED_WR_Byte(0x14, OLED_CMD); // Enable internal charge pump
	OLED_WR_Byte(0xA4, OLED_CMD); // Disable full white screen
	OLED_WR_Byte(0xA6, OLED_CMD); // Non-inverted display
	OLED_Clear();
	OLED_WR_Byte(0xAF, OLED_CMD); // Turn OLED screen on
}
