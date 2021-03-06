/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_EX1.C
 *      Purpose: Your First RTX example program
 *----------------------------------------------------------------------------
 *
 * Copyright (c) 1999-2009 KEIL, 2009-2013 ARM Germany GmbH
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/



/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_conf.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "gpio.h"
//#include "cursor.h"
//#include "graphicObject.h"
#include "LcdHal.h"
//#include "uiframework.h"

//#include "pictures.h"
/** @addtogroup STM32F10x_StdPeriph_Template
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_STM32100B_EVAL
#define MESSAGE1   "STM32 MD Value Line "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "  STM32100B-EVAL    "
#elif defined (USE_STM3210B_EVAL)
#define MESSAGE1   "STM32 Medium Density"
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "   STM3210B-EVAL    "
#elif defined (STM32F10X_XL) && defined (USE_STM3210E_EVAL)
#define MESSAGE1   "  STM32 XL Density  "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "   STM3210E-EVAL    "
#elif defined (USE_STM3210E_EVAL)
#define MESSAGE1   " STM32 High Density "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "   STM3210E-EVAL    "
#elif defined (USE_STM3210C_EVAL)
#define MESSAGE1   " STM32 Connectivity "
#define MESSAGE2   " Line Device running"
#define MESSAGE3   " on STM3210C-EVAL   "
#elif defined (USE_STM32100E_EVAL)
#define MESSAGE1   "STM32 HD Value Line "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "  STM32100E-EVAL    "
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// USART_InitTypeDef USART_InitStructure;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* RTX threads -------------------------------------------------------------*/
/* Forward reference */
void threadX (void const *argument);

/* Thread IDs */
osThreadId main_id;
osThreadId threadX_id;

/* Thread definitions */
osThreadDef(threadX, osPriorityNormal, 1, 0);

/* for sync buffer access */
osMutexId lcd_lcd_mutex;
osMutexDef(lcd_lcd_mutex);
/*----------------------------------------------------------------------------
 *   Thread X
 *---------------------------------------------------------------------------*/
void threadX (void const *argument) {
	int ret;
	for (;;) {
		/* Wait for signal or timeout */
		//osSignalWait(0x0004, osWaitForever);
		//LCD_clear_display(LCD_NO_FRAME_INVERSION);
		osDelay(40);

		LCD_update();

		/* Indicate to main thread completion of do-that */
		//osSignalSet(main_id, 0x0004);

	}
}

/*----------------------------------------------------------------------------
 *   Main Thread
 *---------------------------------------------------------------------------*/

int main (void) {
	//GPIO_Configuration();
	uint16_t x = 100, y= 100;
	//pinMode(GPIOF,GPIO_Pin_6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz,RCC_APB2Periph_GPIOF);

	/* If the LCD Panel has a resolution of 320x240 this command is not needed, it's set by default */
	/* Set_LCD_Resolution( 320, 240 ); */

	/* Initialize the LCD */
	GL_LCD_Init();

	/* Get main thread ID */
	main_id = osThreadGetId();
	// uint32_t tick;

	/* Create thread X */
	threadX_id = osThreadCreate(osThread(threadX), NULL);
	GL_SetTextColor(GL_Black);
	GL_SetBackColor(GL_White);
	//GL_SetBackColor(GL_White);	GL_SetTextColor(GL_Black);



	Point pts [3];
	pts[0].X = 20;
	pts[0].Y = 20;
	pts[1].X = 50;
	pts[1].Y = 100;
	pts[2].X = LCD_PIXEL_WIDTH;
	pts[2].Y = 50;
	pPoint pp = pts;

	GL_Clear(GL_BackColor);
	//LCD_update();
	// effect only strings
	LCD_Change_Direction(_0_degree);
	//LCD_Change_Direction_2(_90_degree);
	//LCD_DrawLine(100,100,100,Horizontal);
	//LCD_DrawLine(200,100,100,Horizontal);
	//LCD_DrawRect(x,y,50,70);
	//LCD_update();
	//LCD_DrawCircle(x,y,50);
	//Set_LCD_Resolution(400,240);
	//GL_SetFont(GL_FONT_BIG);
	/* Indicate to thread X completion of do-this */
	//osSignalSet(threadX_id, 0x0004);
	//GL_Clear(GL_BackColor);

	CursorInit(GL_NULL);
	//GL_Clear(GL_BackColor);


	//Show_HomeScreen();

	GL_Page_TypeDef page1;
	Create_PageObj( &page1 );
	GL_PageControls_TypeDef* pageLabel = NewLabel( "pageLabel", "Graphic Library", GL_RIGHT_VERTICAL, GL_FONT_BIG, GL_Black);
	AddPageControlObj(50,50,pageLabel, &page1);

	//GL_PageControls_TypeDef* Button1 = NewButton( "Button1", "BUTTON1", NullFunc );
	//AddPageControlObj(100,100, Button1, &page1);
//	GL_PageControls_TypeDef* Button2 = NewButton( "Button2", "BUTTON2", NullFunc );
//	AddPageControlObj((uint16_t)((LCD_Width/10)*6),(uint8_t)((LCD_Height/9)*7), Button2, &page1);


	/* Declares & initiates a GraphChart Object and add it with specified coordinates toa Page Object */
	//int16_t MyChartPoints[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24};
	//GL_PageControls_TypeDef* MyGraphChart = NewGraphChart("MyGraphChart", "Hours", "Wh", MyChartPoints, 24, GL_TRUE);
	//AddPageControlObj( 0, 0, MyGraphChart, &page1 );

	page1.ShowPage( &page1, GL_TRUE );




	for (;;) {    /* do-this */

		/* Wait for completion of do-that */
		//osSignalWait(0x0004, osWaitForever);

		//tick = osKernelSysTick();
		//LCD_clear_display(LCD_NO_FRAME_INVERSION);

		//delta = osKernelSysTick() - tick;
		//delta = delta >> 6;
		//delta++;
		//LCD_clear_rows(y- 3 -50,y - 3 );
/*
		GL_Clear(GL_BackColor);
				LCD_DrawRect(x,y,50,70);
				GL_LCD_DrawCircle(x,y,35);
		LCD_PrintStringLine(x+ 5,y,"ABCD");
		//		//LCD_DrawLine(x,y,200,Horizontal);
		LCD_DrawFullRect(x+60,y+60,25,25);
		LCD_DrawFullCircle(x,y,10);
		//
		//		LCD_DrawUniLine(x,y,0,200);
				pts[1].X = x;
				pts[1].Y = y;
		//LCD_PolyLine(pp,3);
		//LCD_ClosedPolyLine(pp,3);
		//LCD_PolyLineRelative(pp,3);
		//LCD_FillPolyLine(pp,3);
		x = x + 4;
		y = y + 2;

		//GL_LCD_DisplayChar(x,y,'A',GL_FALSE);
		//GL_DisplayAdjStringLine(x,y,"ABCD",GL_TRUE);
		//osSignalSet(threadX_id, 0x0004);

				if (y == 240)
				{
					x = 0;
					y = 0;
				}
*/

		osDelay(100);
		//pageLabel->objCoordinates.MinX = x;
		//pageLabel->objCoordinates.MinY = y;
		//RefreshPage(&page1);


	}
}





/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
//int main(void)
//{
//  /*!< At this stage the microcontroller clock setting is already configured,
//       this is done through SystemInit() function which is called from startup
//       file (startup_stm32f10x_xx.s) before to branch to application main.
//       To reconfigure the default setting of SystemInit() function, refer to
//       system_stm32f10x.c file
//     */
//
//  /* Initialize LEDs, Key Button, LCD and COM port(USART) available on
//     STM3210X-EVAL board ******************************************************/
//  STM_EVAL_LEDInit(LED1);
//  STM_EVAL_LEDInit(LED2);
//  STM_EVAL_LEDInit(LED3);
//  STM_EVAL_LEDInit(LED4);
//
//  /* USARTx configured as follow:
//        - BaudRate = 115200 baud
//        - Word Length = 8 Bits
//        - One Stop Bit
//        - No parity
//        - Hardware flow control disabled (RTS and CTS signals)
//        - Receive and transmit enabled
//  */
//  USART_InitStructure.USART_BaudRate = 115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//  STM_EVAL_COMInit(COM1, &USART_InitStructure);
//
//  /* Initialize the LCD */
//#ifdef USE_STM32100B_EVAL
//  STM32100B_LCD_Init();
//#elif defined (USE_STM3210B_EVAL)
//  STM3210B_LCD_Init();
//#elif defined (USE_STM3210E_EVAL)
//  STM3210E_LCD_Init();
//#elif defined (USE_STM3210C_EVAL)
//  STM3210C_LCD_Init();
//#elif defined (USE_STM32100E_EVAL)
//  STM32100E_LCD_Init();
//#endif
//
//  /* Display message on STM3210X-EVAL LCD *************************************/
//  /* Clear the LCD */
//  LCD_Clear(LCD_COLOR_WHITE);
//
//  /* Set the LCD Back Color */
//  LCD_SetBackColor(LCD_COLOR_BLUE);
//  /* Set the LCD Text Color */
//  LCD_SetTextColor(LCD_COLOR_WHITE);
//  LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)MESSAGE1);
//  LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)MESSAGE2);
//  LCD_DisplayStringLine(LCD_LINE_2, (uint8_t *)MESSAGE3);
//
//  /* Retarget the C library printf function to the USARTx, can be USART1 or USART2
//     depending on the EVAL board you are using ********************************/
//  printf("\n\r %s", MESSAGE1);
//  printf(" %s", MESSAGE2);
//  printf(" %s\n\r", MESSAGE3);
//
//  /* Turn on leds available on STM3210X-EVAL **********************************/
//  STM_EVAL_LEDOn(LED1);
//  STM_EVAL_LEDOn(LED2);
//  STM_EVAL_LEDOn(LED3);
//  STM_EVAL_LEDOn(LED4);
//
//  /* Add your application code here
//     */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(EVAL_COM1, (uint8_t) ch);
//
//  /* Loop until the end of transmission */
//  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
//  {}
//
//  return ch;
//}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */
void _exit(int status)
{
	while(1);
}
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
