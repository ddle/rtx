#ifndef GPIO_H
#define GPIO_H

#define HIGH 1
#define LOW  0
#define LSBFIRST 0
#define MSBFIRST 1

extern void GPIO_Configuration(void);
extern void pinMode(GPIO_TypeDef * port, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, uint32_t rcc_per_port);
extern uint8_t digitalRead(GPIO_TypeDef * port, uint16_t pin);
extern void digitalWrite(GPIO_TypeDef * port, uint16_t pin, uint8_t level);
extern void digitalWriteHigh(GPIO_TypeDef * port, uint16_t pin);
extern void digitalWriteLow(GPIO_TypeDef * port, uint16_t pin);
extern void shiftOut(GPIO_TypeDef * dataPort, uint16_t dataPin, GPIO_TypeDef * clockPort ,uint16_t clockPin, uint8_t bitOrder, uint8_t val);

void analogEnable();
void analogDisable();
void analogWriteChannel1(uint16_t data);
void analogWriteChannel2(uint16_t data);

#endif
