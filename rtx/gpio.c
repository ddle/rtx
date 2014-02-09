#include "stm32f10x.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_rcc.h"
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  /* Configure USART1 RX (PA.10) as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure GPIO for LEDs as Output push-pull, APON */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

//  /// PWRON
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//  /// D+PU???
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GPIOG, &GPIO_InitStructure);
//

//  /* Configure PD.03 as input floating, VCC control */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* RIGHT Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource13);*/

  /* LEFT Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource14);*/

  /* DOWN Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);*/

  /* UP Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource15);*/

  /* SEL Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7);*/

  /* KEY Button
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource8);*/
}


uint8_t digitalRead(GPIO_TypeDef * port, uint16_t pin)
{
	return (((port->IDR) & (1<<pin)) && 1);
}

void pinMode(GPIO_TypeDef * port, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, uint32_t rcc_per_port)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(rcc_per_port, ENABLE);

	/*Configure GPIO pin */
	GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_InitStruct.GPIO_Mode = mode;
	GPIO_InitStruct.GPIO_Speed = speed;
	GPIO_Init(port, &GPIO_InitStruct);
}

void digitalWrite(GPIO_TypeDef * port, uint16_t pin, uint8_t level)
{
	if (level == HIGH)
		port->BSRR = pin;
	else
		//port->BSRR |= ( (1 << pin) << 16 ) ;
		port->BRR =  pin;
}

void digitalWriteHigh(GPIO_TypeDef * port, uint16_t pin)
{
	port->BSRR = pin;
}

void digitalWriteLow(GPIO_TypeDef * port, uint16_t pin)
{
	port->BSRR = pin ;
}

/// DAC routines on PA4 and PA5 ---------------------------------------------///

void analogEnable()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef  DAC_InitStructure;

	/// GPIOA Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/// DAC Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/// Once the DAC channel is enabled, the corresponding GPIO pin is automatically
    /// connected to the DAC converter. In order to avoid parasitic consumption,
    /// the GPIO pin should be configured in analog

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* DAC channel Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure );
	DAC_Init(DAC_Channel_2, &DAC_InitStructure );

	/// Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
    /// automatically connected to the DAC converter.
	DAC_Cmd( DAC_Channel_1, ENABLE );
	DAC_Cmd( DAC_Channel_2, ENABLE );
}
void analogDisable()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, DISABLE);
	DAC_Cmd( DAC_Channel_1, DISABLE );
	DAC_Cmd( DAC_Channel_2, DISABLE );
	DAC_SoftwareTriggerCmd( DAC_Channel_1, DISABLE );
	DAC_SoftwareTriggerCmd( DAC_Channel_2, DISABLE );
}

/// Data must be right aligned !!!
void analogWriteChannel1(uint16_t data)
{
	DAC_SetChannel1Data( DAC_Align_12b_R, data );
	DAC_SoftwareTriggerCmd( DAC_Channel_1, ENABLE );
}

void analogWriteChannel2(uint16_t data)
{
	DAC_SetChannel2Data( DAC_Align_12b_R, data );
	DAC_SoftwareTriggerCmd( DAC_Channel_2, ENABLE );
}
/*
 * Description
 * Shifts out a byte of data one bit at a time. Starts from either the most
 * (i.e. the leftmost) or least (rightmost) significant bit. Each bit is written
 * in turn to a data pin, after which a clock pin is pulsed (taken high, then low)
 * to indicate that the bit is available.
 *
 * Params:
 *  + initialized data pin and clock pin
 *  + bitOrder LSBFIRST/MSBFIRST
 *  + val: value to be shifted
 *
 * Ported from Arduino source
 * */
void shiftOut(GPIO_TypeDef * dataPort, uint16_t dataPin, GPIO_TypeDef * clockPort ,uint16_t clockPin, uint8_t bitOrder, uint8_t val)
{
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
		{
			digitalWrite(dataPort, dataPin, !!(val & (1 << i)));
			__NOP();__NOP();
		}
		else
		{
			digitalWrite(dataPort, dataPin, !!(val & (1 << (7 - i))));
			__NOP();__NOP();
		}

		digitalWrite(clockPort, clockPin, HIGH);__NOP();__NOP();
		digitalWrite(clockPort, clockPin, LOW);__NOP();__NOP();
	}
}
