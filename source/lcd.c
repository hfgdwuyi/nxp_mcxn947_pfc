/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "lcd.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "semphr.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Port Me. Start */

/* Macros for FlexIO interfacing the LCD */
#define BOARD_FLEXIO              FLEXIO0
#define BOARD_FLEXIO_CLOCK_FREQ   CLOCK_GetFlexioClkFreq()
#define BOARD_FLEXIO_BAUDRATE_BPS 160000000U

/* Macros for FlexIO shifter, timer, and pins. */
#define BOARD_FLEXIO_WR_PIN           1
#define BOARD_FLEXIO_RD_PIN           0
#define BOARD_FLEXIO_DATA_PIN_START   16
#define BOARD_FLEXIO_TX_START_SHIFTER 0
#define BOARD_FLEXIO_RX_START_SHIFTER 0
#define BOARD_FLEXIO_TX_END_SHIFTER   7
#define BOARD_FLEXIO_RX_END_SHIFTER   7
#define BOARD_FLEXIO_TIMER            0
/* Port Me. End */

#define DEMO_MS_TO_TICK(ms) ((ms * configTICK_RATE_HZ / 1000) + 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
LCD_FontTypeDef	LCD_Font;

uint8_t Font_InputVoltage[] = {
	/*-- ID:0,字符:"V",ASCII编码:56,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x04,0x3c,0xc4,0x00,0x00,0xe4,0x1c,0x04,0x00,0x00,0x03,0x1c,0x07,0x00,0x00,0x00,

	/*-- ID:1,字符:"i",ASCII编码:69,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x40,0x4c,0xcc,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x1f,0x10,0x10,0x00,0x00,

	/*-- ID:2,字符:"n",ASCII编码:6E,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x40,0xc0,0x80,0x40,0x40,0x40,0x80,0x00,0x10,0x1f,0x10,0x00,0x00,0x10,0x1f,0x10,

	/*-- ID:3,字符:":",ASCII编码:3A,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00
};

uint8_t Font_InputCurrent[] = {
	/*-- ID:0,字符:"I",ASCII编码:49,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x04,0x04,0xfc,0x04,0x04,0x00,0x00,0x00,0x10,0x10,0x1f,0x10,0x10,0x00,0x00,

	/*-- ID:1,字符:"i",ASCII编码:69,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x40,0x4c,0xcc,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x1f,0x10,0x10,0x00,0x00,

	/*-- ID:2,字符:"n",ASCII编码:6E,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x40,0xc0,0x80,0x40,0x40,0x40,0x80,0x00,0x10,0x1f,0x10,0x00,0x00,0x10,0x1f,0x10,

	/*-- ID:3,字符:":",ASCII编码:3A,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00
};

uint8_t Font_OutputVoltage[] = {
	/*-- ID:0,字符:"V",ASCII编码:56,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x04,0x3c,0xc4,0x00,0x00,0xe4,0x1c,0x04,0x00,0x00,0x03,0x1c,0x07,0x00,0x00,0x00,

	/*-- ID:1,字符:"o",ASCII编码:6F,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x80,0x40,0x40,0x40,0x40,0x80,0x00,0x00,0x0f,0x10,0x10,0x10,0x10,0x0f,0x00,

	/*-- ID:2,字符:"u",ASCII编码:75,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x40,0xc0,0x00,0x00,0x00,0x40,0xc0,0x00,0x00,0x0f,0x10,0x10,0x10,0x08,0x1f,0x10,

	/*-- ID:3,字符:"t",ASCII编码:74,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x40,0x40,0xf0,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x0f,0x10,0x10,0x08,0x00,

	/*-- ID:4,字符:":",ASCII编码:3A,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00
};

uint8_t Font_OutputCurrent[] = {
	/*-- ID:0,字符:"I",ASCII编码:49,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x04,0x04,0xfc,0x04,0x04,0x00,0x00,0x00,0x10,0x10,0x1f,0x10,0x10,0x00,0x00,

	/*-- ID:1,字符:"o",ASCII编码:6F,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x80,0x40,0x40,0x40,0x40,0x80,0x00,0x00,0x0f,0x10,0x10,0x10,0x10,0x0f,0x00,

	/*-- ID:2,字符:"u",ASCII编码:75,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x40,0xc0,0x00,0x00,0x00,0x40,0xc0,0x00,0x00,0x0f,0x10,0x10,0x10,0x08,0x1f,0x10,

	/*-- ID:3,字符:"t",ASCII编码:74,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x40,0x40,0xf0,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x0f,0x10,0x10,0x08,0x00,

	/*-- ID:4,字符:":",ASCII编码:3A,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00
};


uint8_t Font_0[] = {
		0x00,0xf0,0x08,0x04,0x04,0x08,0xf0,0x00,0x00,0x07,0x08,0x10,0x10,0x08,0x07,0x00
};
uint8_t Font_1[] = {
		0x00,0x00,0x08,0x08,0xfc,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x1f,0x10,0x10,0x00
};
uint8_t Font_2[] = {
		0x00,0x38,0x04,0x04,0x04,0x84,0x78,0x00,0x00,0x18,0x14,0x12,0x11,0x10,0x18,0x00
};
uint8_t Font_3[] = {
		0x00,0x18,0x04,0x84,0x84,0x44,0x38,0x00,0x00,0x0c,0x10,0x10,0x10,0x11,0x0e,0x00
};
uint8_t Font_4[] = {
		0x00,0x00,0xc0,0x20,0x18,0xfc,0x00,0x00,0x00,0x03,0x02,0x12,0x12,0x1f,0x12,0x12
};
uint8_t Font_5[] = {
		0x00,0xfc,0x44,0x44,0x44,0x84,0x04,0x00,0x00,0x0c,0x10,0x10,0x10,0x08,0x07,0x00
};
uint8_t Font_6[] = {
		0x00,0xf0,0x88,0x44,0x44,0x48,0x80,0x00,0x00,0x07,0x08,0x10,0x10,0x10,0x0f,0x00
};
uint8_t Font_7[] = {
		0x00,0x0c,0x04,0x04,0xc4,0x34,0x0c,0x00,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00
};
uint8_t Font_8[] = {
		0x00,0x38,0x44,0x84,0x84,0x44,0x38,0x00,0x00,0x0e,0x11,0x10,0x10,0x11,0x0e,0x00
};
uint8_t Font_9[] = {
		0x00,0xf8,0x04,0x04,0x04,0x88,0xf0,0x00,0x00,0x00,0x09,0x11,0x11,0x08,0x07,0x00
};
uint8_t Font_V[] = {
		0x04,0x3c,0xc4,0x00,0x00,0xe4,0x1c,0x04,0x00,0x00,0x03,0x1c,0x07,0x00,0x00,0x00
};
uint8_t Font_A[] = {
		/*-- ID:0,字符:"A",ASCII编码:41,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
		0x00,0x00,0xe0,0x1c,0x70,0x80,0x00,0x00,0x10,0x1e,0x11,0x01,0x01,0x13,0x1c,0x10
};

uint8_t Font_Point[] = {
	/*-- ID:0,字符:".",ASCII编码:2E,对应字:宽x高=8x16,画布:宽W=8 高H=16,共16字节*/
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00
};

void Display(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(uint16_t n_ms)
{
	uint16_t j,k;
	for(j=0;j<n_ms;j++)
		for(k=0;k<110;k++);
}

//写指令到LCD 模块
void transfer_command_lcd(uint8_t data1)
{
 char i;
 LCD_CS_LOW();
 LCD_RS_LOW();
 for(i=0;i<8;i++)
  {
	 LCD_SCL_LOW();
	 //delay_us(10); //加少量延时
	 if(data1&0x80) LCD_SDA_HIGH();
	 else LCD_SDA_LOW();
	 LCD_SCL_HIGH();
	 //delay_us(10); //加少量延时
	 data1=data1<<1;
  }
 LCD_CS_HIGH();
}

//写数据到LCD 模块
void transfer_data_lcd(uint8_t data1)
{
 char i;
 LCD_CS_LOW();
 LCD_RS_HIGH();
 for(i=0;i<8;i++)
 {
	 LCD_SCL_LOW();
	 if(data1&0x80) LCD_SDA_HIGH();
	 else LCD_SDA_LOW();
	 LCD_SCL_HIGH();
	 data1=data1<<1;
	 }
 LCD_CS_HIGH();
}

void lcd_address(uint32_t page,uint32_t column)
{
 column=column-0x01;
 transfer_command_lcd(0xb0+page-1); //设置页地址，每8 行为一页，全屏共64 行，被分成8 页
 transfer_command_lcd(0x10+(column>>4&0x0f)); //设置列地址的高4 位
 transfer_command_lcd(column&0x0f); //设置列地址的低4 位
}

//显示5X8 点阵图像、ASCII, 或5x8 点阵的自造字符、其他图标
void display_graphic_5x8(uint8_t page,uint8_t column,uint8_t *dp)
{
 uint32_t i;
 lcd_address(page,column);
 for (i=0;i<6;i++)
 {
 transfer_data_lcd(*dp);
 dp++;
 }

}

void display_128x64(uint8_t *dp)
{
 uint8_t i,j;

 for(j=0;j<8;j++)
 {
 lcd_address(j+1,1);
 for (i=0;i<128;i++)
 {
 transfer_data_lcd(*dp); //写数据到LCD,每写完一个8 位的数据后列地址自动加1
 dp++;
 }
 }
}

void display_graphic_16x16(uint8_t page,uint8_t column,uint8_t *dp)
{
	uint8_t i,j;
	 for(j=0;j<2;j++)
	 {
		 lcd_address(page+j,column);
		 for (i=0;i<16;i++)
		 {
			 transfer_data_lcd(*dp); //写数据到LCD,每写完一个8 位的数据后列地址自动加1
			 dp++;
		 }
	 }
}

void display_graphic_8x16(uint8_t page,uint8_t column,uint8_t *dp)
{
	uint8_t i,j;
	 for(j=0;j<2;j++)
	 {
		 lcd_address(page+j,column);
		 for (i=0;i<8;i++)
		 {
			 transfer_data_lcd(*dp); //写数据到LCD,每写完一个8 位的数据后列地址自动加1
			 dp++;
		 }
	 }
}

//送指令到晶联讯字库IC
void send_command_to_ROM( uint8_t datu )
{
	 uint8_t i;
	 for(i=0;i<8;i++ )
	 {
		 LCD_ROM_SCK_LOW();
		 delay(10);
	 if(datu&0x80)LCD_ROM_IN_HIGH();
	 else LCD_ROM_IN_LOW();
	 datu = datu<<1;
	 LCD_ROM_SCK_HIGH();
	 delay(10);
	 }
}

//从晶联讯字库IC 中取汉字或字符数据（1 个字节）
static uint8_t get_data_from_ROM()
{
	uint8_t i;
	uint8_t ret_data=0;
 for(i=0;i<8;i++)
 {
	 LCD_ROM_SCK_LOW();
	 //delay_us(1);
	 ret_data=ret_data<<1;
	//  if( ROM_OUT )
	//  ret_data=ret_data+1;
	//  else
	//  ret_data=ret_data+0;
	 LCD_ROM_SCK_HIGH();
	 //delay_us(1);
 }
 return(ret_data);
}

//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_16x16(uint32_t fontaddr,uint8_t page,uint8_t column)
{
	uint8_t i,j,disp_data;
 LCD_ROM_CS_LOW();
 send_command_to_ROM(0x03);
 send_command_to_ROM((fontaddr&0xff0000)>>16); //地址的高8 位,共24 位
 send_command_to_ROM((fontaddr&0xff00)>>8); //地址的中8 位,共24 位
 send_command_to_ROM(fontaddr&0xff); //地址的低8 位,共24 位
 for(j=0;j<2;j++)
 {
 lcd_address(page+j,column);
 for(i=0; i<16; i++ )
 {
 disp_data=get_data_from_ROM();
 transfer_data_lcd(disp_data); //写数据到LCD,每写完1 字节的数据后列地址自动加1
 }
 }
 LCD_ROM_CS_HIGH();
}

//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_5x8(uint32_t fontaddr,uint8_t page,uint8_t column)
{
	uint8_t i,disp_data;
	LCD_ROM_CS_LOW();
	send_command_to_ROM(0x03);
	send_command_to_ROM((fontaddr&0xff0000)>>16); //地址的高8 位,共24 位
	send_command_to_ROM((fontaddr&0xff00)>>8); //地址的中8 位,共24 位
	send_command_to_ROM(fontaddr&0xff); //地址的低8 位,共24 位
	lcd_address(page,column);
	for(i=0; i<5; i++ )
	{
		disp_data=get_data_from_ROM();
		transfer_data_lcd(disp_data); //写数据到LCD,每写完1 字节的数据后列地址自动加1
	}
	LCD_ROM_CS_HIGH();
}

void display_string_5x8(uint8_t page,uint8_t column,uint8_t *text)
{
	 unsigned char i= 0;
	 uint32_t fontaddr = 0;
	 while((text[i]>0x00))
	 {
		 if((text[i]>=0x20) &&(text[i]<=0x7e))
		 {
			 fontaddr = (text[i]- 0x20);
			 fontaddr = (uint32_t)(fontaddr*8);
			 fontaddr = (uint32_t)(fontaddr+0x3bfc0);


			 get_and_write_5x8(fontaddr,page,column); //从指定地址读出数据写到液晶屏指定（page,column)座标中

			 i+=1;
			 column+=6;
		 }
		 else
			 i++;
	 }
}

//全屏清屏
void clear_screen()
{
 unsigned char i,j;


 for(i=0;i<9;i++)
 {
 transfer_command_lcd(0xb0+i);
 transfer_command_lcd(0x10);
 transfer_command_lcd(0x00);
 for(j=0;j<132;j++)
 {
 transfer_data_lcd(0x00);
 }
 }
}

void lv_port_disp_init(void)
{
	LCD_RESET_LOW();
	delay(100);
	LCD_RESET_HIGH(); //复位完毕
	delay(100);
	transfer_command_lcd(0xe2); //软复位
	delay(5);
	transfer_command_lcd(0x2c); //升压步聚1
	delay(50);
	transfer_command_lcd(0x2e); //升压步聚2
	delay(50);
	transfer_command_lcd(0x2f); //升压步聚3
	delay(5);
	transfer_command_lcd(0x23); //粗调对比度，可设置范围0x20～0x27
	transfer_command_lcd(0x81); //微调对比度
	transfer_command_lcd(0x28); //微调对比度的值，可设置范围0x00～0x3f
	transfer_command_lcd(0xa2); //1/9 偏压比（bias）
	transfer_command_lcd(0xc8); //行扫描顺序：从上到下
	transfer_command_lcd(0xa0); //列扫描顺序：从左到右
	transfer_command_lcd(0x40); //起始行：第一行开始
	transfer_command_lcd(0xaf); //开显示

	clear_screen();

}

void FontDisplay(uint8_t FontMolds[], LCD_FontTypeDef LCD_Font)
{
	uint8_t i;
	uint8_t *p = FontMolds;
	for(i = 0;i < LCD_Font.ubFontSize; i++){
		display_graphic_8x16(LCD_Font.ubFontXAxis,(8*i + LCD_Font.ubFontYAxis),p);
		p = p + 16;
	}
}

void Display(void)
{
	static uint8_t ubLoop = 0;
	static uint8_t Value = 0;
	static uint8_t FontValue1 = 0;
	static uint8_t FontValue2 = 0;
	static uint8_t FontValue3 = 0;

    {
    	Value++;
    	if(Value > 99){
    		Value = 0;
    	}

    	ubLoop++;
		if(ubLoop > 1){
			ubLoop = 0;
		}

    	FontValue1 = Value / 10;
    	FontValue2 = Value % 10;
		ubLoop = 0;
    	switch(ubLoop){
    	case 0:
    		clear_screen();
    		LCD_Font.ubFontSize = (sizeof(Font_InputVoltage) / sizeof(Font_InputVoltage[0])) / 16;
    		LCD_Font.ubFontXAxis = 1;
    		LCD_Font.ubFontYAxis = 1;
    		FontDisplay(Font_InputVoltage,LCD_Font);

    		LCD_Font.ubFontSize = (sizeof(Font_InputCurrent) / sizeof(Font_InputCurrent[0])) / 16;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_InputCurrent,LCD_Font);

			LCD_Font.ubFontSize = (sizeof(Font_OutputVoltage) / sizeof(Font_OutputVoltage[0])) / 16;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputVoltage,LCD_Font);

			LCD_Font.ubFontSize = (sizeof(Font_OutputCurrent) / sizeof(Font_OutputCurrent[0])) / 16;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputCurrent,LCD_Font);


//Vin value
			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 112;
			FontDisplay(Font_7,LCD_Font);

    		LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 104;
			FontDisplay(Font_Point,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 104 - 8;
			FontDisplay(Font_0,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 104 - 2*8;
			FontDisplay(Font_2,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 104 - 3*8;
			FontDisplay(Font_2,LCD_Font);
			

//Iin value
			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 112;
			FontDisplay(Font_3,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 112 - 8;
			FontDisplay(Font_Point,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 112 - 2*8;
			FontDisplay(Font_5,LCD_Font);


//Vout value
			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 112;
			FontDisplay(Font_7,LCD_Font);

    		LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 104;
			FontDisplay(Font_Point,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 104 - 8;
			FontDisplay(Font_0,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 104 - 2*8;
			FontDisplay(Font_2,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 104 - 3*8;
			FontDisplay(Font_2,LCD_Font);
			

//Iout value
			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 112;
			FontDisplay(Font_3,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 112 - 8;
			FontDisplay(Font_Point,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 112 - 2*8;
			FontDisplay(Font_5,LCD_Font);


			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 120;
			FontDisplay(Font_V,LCD_Font);


			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 120;
			FontDisplay(Font_A,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 120;
			FontDisplay(Font_A,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 120;
			FontDisplay(Font_V,LCD_Font);

			LCD_Font.ubFontSize = 1;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 120;
			FontDisplay(Font_A,LCD_Font);


    		break;
    	case 1:
    		clear_screen();
    		LCD_Font.ubFontSize = (sizeof(Font_OutputVoltage) / sizeof(Font_OutputVoltage[0])) / 16;
			LCD_Font.ubFontXAxis = 1;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputVoltage,LCD_Font);

			LCD_Font.ubFontSize = (sizeof(Font_OutputVoltage) / sizeof(Font_OutputVoltage[0])) / 16;
			LCD_Font.ubFontXAxis = 3;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputVoltage,LCD_Font);

			LCD_Font.ubFontSize = (sizeof(Font_OutputVoltage) / sizeof(Font_OutputVoltage[0])) / 16;
			LCD_Font.ubFontXAxis = 5;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputVoltage,LCD_Font);

			LCD_Font.ubFontSize = (sizeof(Font_OutputVoltage) / sizeof(Font_OutputVoltage[0])) / 16;
			LCD_Font.ubFontXAxis = 7;
			LCD_Font.ubFontYAxis = 1;
			FontDisplay(Font_OutputVoltage,LCD_Font);

			break;
    	case 2:
    		clear_screen();

			break;
    	case 3:
    		clear_screen();

			break;
    	default:
    		break;

    	}
    }
}
