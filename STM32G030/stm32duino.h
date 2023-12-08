#ifndef _STM32DUINO_H
#define _STM32DUINO_H

//STM32duino code
// - using STM32G030F / STM32G031F chips
// - free running systick for ticks
// - 
// - version history
// - v0.1a, 02/11/2023: initial port from stm32f0 code base. GPIO + ticks work
// - v0.1b, 03/04/2023: timers, pwm, uart and rtc now working
// - v0.1c, 03/25/2023: added support for chained (32-bit) timers
// - v0.1d, 03/19/2023: streamlined the pwm initialization code
// - v0.1e, 08/23/2023: refined lptim1/2 and lpuart code
// - v0.1f, 10/21/2023: added support for tamper / backup registers (5 of them)
//
//
//               STM32G030F/031F
//      Vdd     |=====================|
//       |      |                     |
//       +------| Vdd            PA14 |<----- SWCLK (Pin20)
//              |                     |
//       +------| BOOT0          PA13 |<----> SWDIO (Pin19)
//              |                     |
//              |                     |
// GND<---------| Vss             PA7 |--+--> GND/LED1
//              |                     |  |
//              |                 PA6 |--+--> LED
//              |                     |
//              |                 PA3 |>----> PWR
//              |                     |
//              |                 PA2 |>----> U2TX/LED2
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |=====================|
//
//
//
//
//

#include <stm32g0xx.h>						//we use STM32G030
#include <string.h>							//we use strcpy()
#include <stdint.h>							//we use uint types
#include <time.h>							//we use time.h

//hardware configuration
#define USE_CORETICK						//use coretick/DWT for tick generation - ignored for STM32G0
#define uRAM_SIZE			40				//uart buffer size

//oscillator configuration
#define F_XTAL				8000000ul		//crystal frequency, user-specified
#define F_LSE				32768ul			//crystal frequency, user-specified

//configuration for circular buffer
/*******NOTE********************************/
/******cBUFF_SIZE: must be power of 2*******/
/*******************************************/
#define cBUFF_SIZE 			32				//must be power of 2: 0..256 -> for buffers with more than 256 elements, change data type for writeIndex/readIndex
typedef char cbuffer_t;						//storage unit for the buffer
//end configuration for circular buffer

//pin configuration
//uart1/2/3 pins
#define U1TX2PIN()					U1TX2PB6()		//u1tx to pa9, pb6
//#define U1RX2PIN()					U1RX2PA10()		//u1rx to pa10,pb7
#define U2TX2PIN()					U2TX2PA2()		//u2tx to pa2, pa14
//#define U2RX2PIN()					U2RX2PA3()		//u2rx to pa3, pa15
#define LPU1TX2PIN()				LPU1TX2PA2()		//PA2, PB11
#define LPU1RX2PIN()				LPU1RX2PA3()		//PA3, PB10

//timer pins:
//pwm output pins
//tim1
//#define TIM1CH1toPIN()			TIM1CH1toPA8()		//PA8
//#define TIM1CH2toPIN()			TIM1CH2toPB3()		//PB3, PA9
#define TIM1CH3toPIN()				TIM1CH3toPA10()		//PB6, PA10
//#define TIM1CH4toPIN()			TIM1CH4toPA11()		//PA11
//#define TIM1CH1NtoPIN()			TIM1CH1NtoPA7()		//PA7, PB13
//#define TIM1CH2NtoPIN()			TIM1CH2NtoPB14()	//PB0, PB14
//#define TIM1CH3NtoPIN()			TIM1CH3NtoPB15()	//PB1, PB15

//tim3
//#define TIM3CH1toPIN()				TIM3CH1toPA6()		//PA6, PB4
//#define TIM3CH2toPIN()				TIM3CH2toPA7()		//PA7, PB5
#define TIM3CH3toPIN()				TIM3CH3toPB0()		//PB0
//#define TIM3CH4toPIN()			TIM3CH4toPB1()		//PB1

//tim14
#define TIM14CH1toPIN()				TIM14CH1toPA4()		//PB1, PA4, PA7
//tim16
#define TIM16CH1toPIN()				TIM16CH1toPA6()		//PB8, PA6
//#define TIM16CH1NtoPIN()			TIM16CH1NtoPB6()	//PB6
//tim17
#define TIM17CH1toPIN()				TIM17CH1toPA7()		//PB9, PA7
//#define TIM17CH1NtoPIN()			TIM17CH1NtoPB7()	//PB7

//spi pins
//spi1 pins
#define SCK1toPIN()					SCK1toPA1()			//PA1, PA5, PB3, 
#define MOSI1toPIN()				MOSI1toPA2()		//PA2, PA7, PA12, PB5, 
#define MISO1toPIN()				MISO1toPA6()		//PA6, PA11, PB4, 
#define NSS1toPIN()					NSS1toPA4()			//PA4, PA15, PB0, 
//spi2 pins
#define SCK2toPIN()					SCK2toPA0()			//PA0, PB13, 
#define MOSI2toPIN()				MOSI2toPA3()		//PA3, PA8, PB11, PB15, 
#define MISO2toPIN()				MISO2toPB14()		//PB14, 
#define NSS2toPIN()					NSS2toPB12()		//PB12, 

//i2c pins
#define SCL1toPIN()					SCL1toPA9()			//PA9, PB6, PB8, PB10
#define SDA1toPIN()					SDA1toPA10()		//PA10,PB7, PB9, PB11

//mco
#define MCOtoPIN()					MCOtoPA9()			//PA8, PA9
//end user specification

//pin mapping
//u1tx/rx pins
#define U1TX2PB6()					pinAFIO(PB6, AFIO0)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOB->OTYPER &=~(1<< 6); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 6%8)))) | (0<<(4*( 6%8)));} while (0)		//U1TX to Pb6, AF0
#define U1RX2PB7()					pinAFIO(PB7, AFIO0)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOB->OTYPER &=~(1<< 7); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 7%8)))) | (0<<(4*( 7%8)));} while (0)		//U1RX to Pb7 ,AF0)
#define U1TX2PA9()					pinAFIO(PA9, AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (1<<(4*( 9%8)));} while (0)		//U1TX to PA9, AF1
#define U1RX2PA10()					pinAFIO(PA10,AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (1<<(4*(10%8)));} while (0)		//U1RX to PA10,AF1)
//u2tx/rx pins
#define U2TX2PA2()					pinAFIO(PA2, AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 2))) | (/*0b10*/0x02<<(2* 2)); GPIOA->OTYPER &=~(1<< 2); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 2))) | (/*0b01*/0x01<<(2* 2)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 2%8)))) | (1<<(4*( 2%8)));} while (0)		//U2TX to PA2, AF1
#define U2RX2PA3()					pinAFIO(PA3, AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 3))) | (/*0b10*/0x02<<(2* 3)); GPIOA->OTYPER &=~(1<< 3); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 3))) | (/*0b01*/0x01<<(2* 3)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 3%8)))) | (1<<(4*( 3%8)));} while (0)		//U2RX to PA3, AF1)
#define U2TX2PA14()					pinAFIO(PA13,AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*14))) | (/*0b10*/0x02<<(2*14)); GPIOA->OTYPER &=~(1<<14); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*14))) | (/*0b01*/0x01<<(2*14)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(14%8)))) | (1<<(4*(14%8)));} while (0)		//U2TX to PA14,AF1
#define U2RX2PA15()					pinAFIO(PA15,AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOA->OTYPER &=~(1<<15); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(15%8)))) | (1<<(4*(15%8)));} while (0)		//U2RX to PA15,AF1)
//LPUART1 pins
#define LPU1TX2PA2()				pinAFIO(PA2, AFIO6)
#define LPU1RX2PA3()				pinAFIO(PA3, AFIO6)
#define LPU1TX2PB11()				pinAFIO(PB11,AFIO1)
#define LPU1RX2PB10()				pinAFIO(PB10,AFIO1)

//pwm output pins
//tim1
#define TIM1CH1toPA8()				pinAFIO(PA8, AFIO2)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 8))) | (/*0b10*/0x02<<(2* 8)); GPIOA->OTYPER &=~(1<< 8); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 8))) | (/*0b01*/0x01<<(2* 8)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM1CH2toPA9()				pinAFIO(PA9, AFIO2)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM1CH2toPB3()				pinAFIO(PB3, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 3))) | (/*0b10*/0x02<<(2* 3)); GPIOB->OTYPER &=~(1<< 3); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 3))) | (/*0b01*/0x01<<(2* 3)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 3%8)))) | (1<<(4*( 3%8)));} while (0)
#define TIM1CH3toPB6()				pinAFIO(PB6, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOB->OTYPER &=~(1<< 6); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 6%8)))) | (1<<(4*( 6%8)));} while (0)
#define TIM1CH3toPA10()				pinAFIO(PA10,AFIO2)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (2<<(4*(10%8)));} while (0)
#define TIM1CH4toPA11()				pinAFIO(PA11,AFIO2)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*11))) | (/*0b10*/0x02<<(2*11)); GPIOA->OTYPER &=~(1<<11); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*11))) | (/*0b01*/0x01<<(2*11)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(11%8)))) | (2<<(4*(11%8)));} while (0)
#define TIM1CH1NtoPA7()				pinAFIO(PA7, AFIO2)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)
#define TIM1CH1NtoPB13()			pinAFIO(PB13,AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*13))) | (/*0b10*/0x02<<(2*13)); GPIOB->OTYPER &=~(1<<13); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*13))) | (/*0b01*/0x01<<(2*13)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(13%8)))) | (2<<(4*(13%8)));} while (0)
#define TIM1CH2NtoPB14()			pinAFIO(PB14,AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*14))) | (/*0b10*/0x02<<(2*14)); GPIOB->OTYPER &=~(1<<14); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*14))) | (/*0b01*/0x01<<(2*14)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(14%8)))) | (2<<(4*(14%8)));} while (0)
#define TIM1CH2NtoPB0()				pinAFIO(PB0, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 0))) | (/*0b10*/0x02<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 0))) | (/*0b01*/0x01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (2<<(4*( 0%8)));} while (0)
#define TIM1CH3NtoPB1()				pinAFIO(PB1, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (2<<(4*( 1%8)));} while (0)
#define TIM1CH3NtoPB15()			pinAFIO(PB15,AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOB->OTYPER &=~(1<<15); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(15%8)))) | (2<<(4*(15%8)));} while (0)

//tim3
#define TIM3CH1toPA6()				pinAFIO(PA6, AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (1<<(4*( 6%8)));} while (0)
#define TIM3CH1toPB4()				pinAFIO(PB4, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 4))) | (/*0b10*/0x02<<(2* 4)); GPIOB->OTYPER &=~(1<< 4); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 4))) | (/*0b01*/0x01<<(2* 4)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 4%8)))) | (1<<(4*( 4%8)));} while (0)
#define TIM3CH2toPB5()				pinAFIO(PB5, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 5))) | (/*0b10*/0x02<<(2* 5)); GPIOB->OTYPER &=~(1<< 5); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 5))) | (/*0b01*/0x01<<(2* 5)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 5%8)))) | (1<<(4*( 5%8)));} while (0)
#define TIM3CH2toPA7()				pinAFIO(PA7, AFIO1)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (1<<(4*( 7%8)));} while (0)
#define TIM3CH3toPB0()				pinAFIO(PB0, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 0))) | (/*0b10*/0x02<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 0))) | (/*0b01*/0x01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (1<<(4*( 0%8)));} while (0)
#define TIM3CH4toPB1()				pinAFIO(PB1, AFIO1)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (1<<(4*( 1%8)));} while (0)

//tim14
#define TIM14CH1toPB1()				pinAFIO(PB1, AFIO0)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (0<<(4*( 1%8)));} while (0)
#define TIM14CH1toPA4()				pinAFIO(PA4, AFIO4)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 4))) | (/*0b10*/0x02<<(2* 4)); GPIOA->OTYPER &=~(1<< 4); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 4))) | (/*0b01*/0x01<<(2* 4)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 4%8)))) | (2<<(4*( 4%8)));} while (0)
#define TIM14CH1toPA7()				pinAFIO(PA7, AFIO4)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (4<<(4*( 7%8)));} while (0)

//tim16
#define TIM16CH1toPA6()				pinAFIO(PA6, AFIO5)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (5<<(4*( 6%8)));} while (0)
#define TIM16CH1toPB8()				pinAFIO(PB8, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 8))) | (/*0b10*/0x02<<(2* 8)); GPIOB->OTYPER &=~(1<< 8); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 8))) | (/*0b01*/0x01<<(2* 8)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM16CH1NtoPB6()			pinAFIO(PB6, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOB->OTYPER &=~(1<< 6); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 6%8)))) | (2<<(4*( 6%8)));} while (0)

//tim17
#define TIM17CH1toPB9()				pinAFIO(PB9, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOB->OTYPER &=~(1<< 9); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM17CH1toPA7()				pinAFIO(PA7, AFIO5)	//do {GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (5<<(4*( 7%8)));} while (0)
#define TIM17CH1NtoPB7()			pinAFIO(PB7, AFIO2)	//do {GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOB->OTYPER &=~(1<< 7); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)

//spi pins
#define SCK2toPA0()					pinAFIO(PA0, AFIO0)
#define SCK1toPA1()					pinAFIO(PA1, AFIO0)
#define MOSI1toPA2()				pinAFIO(PA2, AFIO0)
#define MOSI2toPA3()				pinAFIO(PA3, AFIO0)
#define NSS1toPA4()					pinAFIO(PA4, AFIO0)
#define SCK1toPA5()					pinAFIO(PA5, AFIO0)
#define MISO1toPA6()				pinAFIO(PA6, AFIO0)
#define MOSI1toPA7()				pinAFIO(PA7, AFIO0)
#define MOSI2toPA8()				pinAFIO(PA8, AFIO0)
#define MISO1toPA11()				pinAFIO(PA11,AFIO0)
#define MOSI1toPA12()				pinAFIO(PA12,AFIO0)
#define SWDIOtoPA13()				pinAFIO(PA13,AFIO0)
#define SWCLKtoPA14()				pinAFIO(PA14,AFIO0)
#define NSS1toPA15()				pinAFIO(PA15,AFIO0)
#define NSS1toPB0()					pinAFIO(PB0, AFIO0)
#define SCK1toPB3()					pinAFIO(PB3, AFIO0)
#define MISO1toPB4()				pinAFIO(PB4, AFIO0)
#define MOSI1toPB5()				pinAFIO(PB5, AFIO0)
#define MOSI2toPB11()				pinAFIO(PB11,AFIO0)
#define NSS2toPB12()				pinAFIO(PB12,AFIO0)
#define SCK2toPB13()				pinAFIO(PB13,AFIO0)
#define MISO2toPB14()				pinAFIO(PB14,AFIO0)
#define MOSI2toPB15()				pinAFIO(PB15,AFIO0)

//i2c pins
#define SCL1toPA9()					pinAFIO(PA9, AFIO4)
#define SCL1toPB6()					pinAFIO(PB6, AFIO1)
#define SCL1toPB8()					pinAFIO(PB8, AFIO1)
#define SCL1toPB10()				pinAFIO(PB10,AFIO1)		//x4 and x6 devices only
#define SDA1toPA10()				pinAFIO(PA10,AFIO4)
#define SDA1toPB7()					pinAFIO(PB7, AFIO1)
#define SDA1toPB9()					pinAFIO(PB9, AFIO1)
#define SDA1toPB11()				pinAFIO(PB11,AFIO1)		//x4 and x6 devices only

//OTHERS
#define MCOtoPA8()					pinAFIO(PA8, AFIO0)		//pa8
#define MCOtoPA9()					pinAFIO(PA9, AFIO0)		//pa9
//end pin configuration

#define F_HSI16		16000000ul			//nominal frequency for HSI16. 16Mhz
#define F_HSE		F_XTAL				//nominal frequency for HSE. user specified
#define F_HSI		F_HSI16				//for compatability
#define F_LSI		32065ul				//nominal frequency for LSI, uncalibrated at 32Khz
#define F_CPU		F_HCLK				//CPU driven off of HCLK
#define F_PHB		F_PCLK				//pheriphral on apb
#define F_SYSCLK	SystemCoreClock		//System Clock
#define F_HCLK		AHBClock()			//AHB clock
#define F_HCLK8		(F_HCLK / 8)		//drives SysTick
#define F_PCLK		APBClock()			//APB clock
uint32_t F_PLLIN(void);					//pll input frequency
#define PLLM		((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos)
#define PLLN		((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)
#define PLLR		((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos)
#define PLLQ		((RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos)
#define PLLP		((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos)
#define F_VCO		(F_PLLIN() * PLLN / (1+PLLM))		//PLL output clock
#define F_PLLRCLK	(F_VCO / (1+PLLR))
#define F_PLLQCLK	(F_VCO / (1+PLLQ))
#define F_PLLPCLK	(F_VCO / (1+PLLP))
#define F_TIMPCLK	(F_PCLK * ((RCC->CFGR & RCC_CFGR_PPRE)?2:1))	//=TIMPCLK: clock for timers
#define F_UART1		F_PCLK	//u1FreqGet()							//get uart1 independent clock rate
#define F_UART2		F_PCLK	//(((RCC->CCIPR & RCC_CCIPR_USART2SEL) == (0<< 2))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART2SEL) == (1<< 2))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART2SEL) == (2<< 2))?F_HSI16:F_LSE)))
#define F_UART3		F_PCLK	//(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (0<< 4))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (1<< 4))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (2<< 4))?F_HSI16:F_LSE)))
#define F_LPUART	F_PCLK	//(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (0<< 4))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (1<< 4))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (2<< 4))?F_HSI16:F_LSE)))
#define F_LPUART1	F_PCLK	//(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (0<< 4))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (1<< 4))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART3SEL) == (2<< 4))?F_HSI16:F_LSE)))
#define F_I2C1		(((RCC->CCIPR & RCC_CCIPR_I2C1SEL  ) == (0<<12))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_I2C1SEL  ) == (1<<12))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_I2C1SEL  ) == (2<<12))?F_HSI16:0)))
#define F_I2C2		(((RCC->CCIPR & RCC_CCIPR_I2S1SEL  ) == (0<<12))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_I2S1SEL  ) == (1<<12))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_I2S1SEL  ) == (2<<12))?F_HSI16:0)))
#define F_TIM1		F_TIMPCLK	//(((RCC->CCIPR & RCC_CCIPR_TIM1SEL  ) == (0<<22))?F_TCLK:F_PLLQCLK)
#define F_TIM15		F_TIMPCLK	//(((RCC->CCIPR & RCC_CCIPR_TIM15SEL ) == (0<<24))?F_TCLK:F_PLLQCLK)
#define F_ADC		(((RCC->CCIPR & RCC_CCIPR_ADCSEL   ) == (0ul<<30))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_ADCSEL   ) == (1ul<<30))?F_PLLPCLK:(((RCC->CCIPR & RCC_CCIPR_ADCSEL  ) == (2ul<<30))?F_HSI16:0)))
#define PWM_PR		0xffff						//pwm period - don't change

//port manipulation macros for PIC.
//MODE:
//Output modes: GPIOMODE_OUTPP, GPIOMODE_OUTOD
//Input mode: GPIOMODE_INPUT, GPIOMODE_INPUTPD, GPIOMODE_INPUTPU
//Alternate function: GPIOMODE_AFPP, GPIOMODE_AFOD
//for moder
#define GPIOMODE_INPUT		(0<<0)		//(0<<0)
#define GPIOMODE_OUTPUT		(1<<0)		//(1<<0)
#define GPIOMODE_AF			(2<<0)
#define GPIOMODE_AN			(3<<0)
//for otyper
#define GPIOMODE_PP			(1<<3)
#define GPIOMODE_OD			(0<<3)
//for pupdr
#define GPIOMODE_FL			(0<<4)
#define GPIOMODE_PU			(1<<4)
#define GPIOMODE_PD			(2<<4)

//AF defs
#define GPIOMODE_AF0		0
#define GPIOMODE_AF1		1
#define GPIOMODE_AF2		2
#define GPIOMODE_AF3		3
#define GPIOMODE_AF4		4
#define GPIOMODE_AF5		5
#define GPIOMODE_AF6		6
#define GPIOMODE_AF7		7

#define GPIOMODE_OUTPP		(GPIOMODE_OUTPUT | GPIOMODE_PP)		//gpio, output, push-pull
#define GPIOMODE_OUTOD		(GPIOMODE_OUTPUT | GPIOMODE_OD)		//gpio, output, od
#define GPIOMODE_INFL		(GPIOMODE_INPUT)
#define GPIOMODE_INPU		(GPIOMODE_INPUT | GPIOMODE_PU)
#define GPIOMODE_INPD		(GPIOMODE_INPUT | GPIOMODE_PD)
#define GPIOMODE_AFPP		(GPIOMODE_AF | GPIOMODE_PP)
#define GPIOMODE_AFOD		(GPIOMODE_AF | GPIOMODE_OD)
#define GPIOMODE_INAN		(GPIOMODE_AN)

//global variables

//tiny scheduler macro
#define TS_RUN_WHILE(cs)	if (cs)						//tiny scheduler macro
#define TSwhile(cs)			TS_RUN_WHILE(cs)			//shorter macro

//operate on a var
#define REG_EQU(var, bits)					var  = (bits)
#define REG_SET(var, bits)					var |= (bits)
#define REG_CLR(var, bits)					var &=~(bits)
#define REG_FLP(var, bits)					var ^= (bits)
#define REG_GET(var, bits)					((var) & (bits))
#define REG_MOD(var, bits, mask)			(var = ((var) &~(mask)) | ((bits) & (mask)))

//port/gpio oriented macros
#define IO_SET(port, pins)					REG_SET(port->ODR, pins)				//set bits on port
#define IO_CLR(port, pins)					REG_CLR(port->ODR, pins)				//clear bits on port
#define IO_FLP(port, pins)					REG_FLP(port->ODR, pins)				//flip bits on port
#define IO_GET(port, pins)					REG_GET(port->IDR, pins)			//return bits on port
//set a pin to output/input
#define IO_OUTPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTPP)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
#define IO_OUTOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTOD)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define IO_OUT(port, pins)					IO_OUTPP(port, pins)					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define IO_INFL(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INFL)		//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define IO_INPU(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPU)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_INPD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPD)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_IN(port, pins)					IO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)
#define IO_AFPP(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFPP)		//configure pin for alternative function output, push-pull, 10Mhz
#define IO_AFOD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFOD)		//configure pin for alternative function output, open-drain, 10Mhz
#define IO_AN(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INAN)

//fast port/gpio oriented macros
#define FIO_SET(port, pins)					port->BSRR = (pins)				//set bits on port
#define FIO_CLR(port, pins)					port->BRR  = (pins)				//clear bits on port
#define FIO_FLP(port, pins)					IO_FLP(port, pins)				//port->ODR ^= (pins)				//flip bits on port
#define FIO_GET(port, pins)					IO_GET(port, pins)				//((port->IDR) & (pins))			//return bits on port

//fast routines through BRR/BSRR registers
#define GIO_SET(port, pins)					FIO_SET(port, pins)				//IO_SET(port, pins)
#define GIO_CLR(port, pins)					FIO_CLR(port, pins)				//IO_CLR(port, pins)	
#define GIO_FLP(port, pins)					IO_FLP(port, pins)
#define GIO_GET(port, pins)					IO_GET(port, pins)
#define GIO_IN(port, pins)					IO_IN(port, pins)
#define GIO_OUT(port, pins)					IO_OUT(port, pins)
//configure gpio mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * gpio, uint32_t mask, uint32_t mode);

//bit-banding on peripherals
//optional on M0/M0+ -> doesn't work on PY32
#define PERI_BASE			PERIPH_BASE		//0x40000000
#define PERI_BB_BASE		(PERI_BASE + 0x02000000)
#define BIO2BB(addr, bit)	( (volatile uint32_t *) (PERI_BB_BASE | (((addr) - PERI_BASE) << 5) | ((bit) << 2)))
#define BIO_GET(port, bit)	(*BIO2BB((uint32_t) & (port->IDR), bit))
#define BIO_SET(port, bit)	(*BIO2BB((uint32_t) & (port->ODR), bit) = 1)
#define BIO_CLR(port, bit)	(*BIO2BB((uint32_t) & (port->ODR), bit) = 0)
#define BIO_FLP(port, bit)	( BIO_GET(port, bit)?BIO_CLR(port, bit):BIO_SET(port, bit))

#define NOP()				Nop()                           //asm("nop")					//nop()
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP24()				{NOP16(); NOP8();}
#define NOP32()				{NOP16(); NOP16();}
#define NOP40()				{NOP32(); NOP8();}
#define NOP64()				{NOP32(); NOP32();}

#define sleep()				asm("sleep")					//put the mcu into sleep

#ifndef ei
#define ei()				//asm volatile ("ei")				//__builtin_enable_interrupts()	//do {INTEnableInterrupts();	INTEnableSystemMultiVectoredInt();} while (0)	//__builtin_enable_interrupts()
#endif

#ifndef di
#define di()				//asm volatile ("di")				//__builtin_enable_interrupts()	//INTDisableInterrupts()			//__builtin_disable_interrupts()	//
#endif

//simple multiples
#define x1(val)				(val)								//multiply val by 1
#define x2(val)				(((val) << 1))						//multiply val by 2
#define x3(val)				(x2(val) + (val))					//multiply val by 3
#define x4(val)				(((val) << 2))						//multiply val by 4
#define x5(val)				(x4(val) + (val))					//multiply val by 5
#define x6(val)				(x4(val) + x2(val))					//multiply val by 6
#define x7(val)				(x6(val) + (val))					//multiply val by 7
#define x8(val)				((val) << 3)						//multiply val by 8
#define x9(val)				(x8(val) + (val))					//multiply val by 9

//multiples of 10s
#define x10(val)			(x8(val) + x2(val))					//multiply val by 10
#define x100(val)			(x10(x10(val)))						//multiply val by 100
#define x1000(val)			(x100(x10(val)))					//multiply val by 1000
#define x1k(val)			x1000(val)							//multiply val by 1000
#define x10k(val)			(x100(x100(val)))					//multiply val by 10000

#define x20(val)			(x2(x10(val)))
#define x30(val)			(x3(x10(val)))
#define x40(val)			(x4(x10(val)))
#define x50(val)			(x5(x10(val)))
#define x60(val)			(x6(x10(val)))
#define x70(val)			(x7(x10(val)))
#define x80(val)			(x8(x10(val)))
#define x90(val)			(x9(x10(val)))

//multiples of 100s
#define x200(val)			(x2(x100(val)))
#define x300(val)			(x3(x100(val)))
#define x400(val)			(x4(x100(val)))
#define x500(val)			(x5(x100(val)))
#define x600(val)			(x6(x100(val)))
#define x700(val)			(x7(x100(val)))
#define x800(val)			(x8(x100(val)))
#define x900(val)			(x9(x100(val)))

//custom definitions
#define x34(val)			(x30(val) + x4(val))				//multiply val by 34
#define x97(val)			(x90(val) + x7(val))				//multiply val by 97x

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
//typedef unsigned long uint32_t;
//typedef signed long int32_t;

//device signature
#define DEV_UID0			(*(uint32_t *)0x1FFF7590)			//uid0..31
#define DEV_UID1			(*(uint32_t *)(&DEV_UID0 + 4))		//uid32..63
#define DEV_UID2			(*(uint32_t *)(&DEV_UID1 + 8))		//uid64..95
#define DEV_FLASHKB			(*(uint16_t *)0x1FFF75E0)			//flash size in KB. 0x40=64KB
#define DEV_PKG				((*(uint32_t *)0x1FFF7500) & 0x0f)	//flash size in KB. 0x40=64KB

//port structure

//global defines

//pin enum - matches GPIO_PinDef[]
typedef enum {
#if defined(GPIOA)
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
#endif		//gpioa
#if defined(GPIOB)
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#endif		//gpiob
#if defined(GPIOC)
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
#endif		//GPIOC
#if defined(GPIOD)
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
#endif		//GPIOD
#if defined(GPIOE)
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
#endif		//GPIOE
#if defined(GPIOF)
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
#endif		//GPIOF
#if defined(GPIOG)
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
#endif		//GPIOG
	PMAX
} PIN_TypeDef;

//map pin number to GPIOx
typedef struct {
	GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;						//pin mask - 16-bit port
} PIN2GPIO;
	
#define INPUT				0
#define OUTPUT				1			//(!INPUT)
#define INPUT_PULLUP		2
#define INPUT_PULLDOWN		3
#define OUTPUT_OD			4
#define INPUT_PU			INPUT_PULLUP
#define INPUT_PD			INPUT_PULLDOWN

#define LOW					0
#define HIGH				(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				1
#define FALLING 			2
#define RISING 				3

#ifndef min
#define min(a,b) 			((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) 			((a)>(b)?(a):(b))
#endif
//#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMillisecond() 	( F_CPU / 1000L )
#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//GPIO
typedef enum {
	//DEFAULT,					//default / reset state
	AFIO0 = 0,					//afio numbers
	AFIO1,
	AFIO2,
	AFIO3,
	AFIO4,
	AFIO5,
	AFIO6,
	AFIO7,
} AFIO_TypeDef;
void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
int digitalRead(PIN_TypeDef pin);
#define pinFlip(pin)		digitalWrite(pin, !digitalRead(pin))

//do not put SWD pins (PA13/14) into GPIO - you will lose debug capability
//if you do, recover as follows:
// 1. pull down the nRST pin and hold it down.
// 2. flash the chip
// 3. release the nRST pin
//
// step 2 and 3 should be done reasonably quickly (within seconds of each other)
//
void pinAFIO(PIN_TypeDef pin, AFIO_TypeDef AFIOn);		//configure pin for AFIO

#define PA11Remap2PA9()		SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP			//remap pa11 to pa9
#define PA11NoRemap()		SYSCFG->CFGR1 &=~SYSCFG_CFGR1_PA11_RMP			//no remap
#define PA12Remap2PA10()	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA12_RMP			//remap pa12 to pa10
#define PA12NoRemap()		SYSCFG->CFGR1 &=~SYSCFG_CFGR1_PA12_RMP			//no remap

//time base
//uint32_t ticks(void);								//timer ticks from timer2
uint32_t systicks(void);
#define ticks()				systicks()				//map ticks to systicks()
#define millis()			(ticks() / cyclesPerMillisecond())
#define micros()			(ticks() / cyclesPerMicrosecond())
void delayTks(uint32_t tks);						//delay a given number of ticks
//void delayUs(uint32_t us);						//delay a given number of micro seconds
//void delayMs(uint32_t ms);						//delay a given number of millie seconds
#define delayUs(us)				delayTks((us) * cyclesPerMicrosecond())
#define delayMs(ms)				delayTks((ms) * cyclesPerMillisecond())
#define delay(ms)				delayMs(ms)
#define delayMilliseconds(ms)	delayMs(ms)
#define delayMicroseconds(us)	delayUs(us)
#define cyclesPerMicrosecond()			(F_CPU / 1000000ul)
#define cyclesPerMillisecond()			(F_CPU / 1000)

//advanced IO
//void tone(void);									//tone frequency specified by F_TONE in STM8Sduino.h
//void noTone(void);
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);		//wait for a pulse and return timing

//pwm output
//dc = 0x00..0x0fff for pwm2/3/4/5, 0x00..0xffff for pwm1
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
void analogWrite(uint8_t pin, uint16_t dc);

//analog read on ADC1
//read DRL first for right aligned results
//uint16_t analogRead(uint8_t pin);

//analog reference - default to AVdd-AVss
//Vref sources: 0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
//void analogReference(uint8_t Vref);

//interrupts
//install external interrupt handler
//mode 1: falling edge, 0: rising edge
//void attachInterrupt(uint8_t intx, void (*isrptr) (void), uint8_t mode);
//void detachInterrupt(uint8_t intx);

//change notification interrupts
//install user CN interrupt handler
//void attachCNInterrupt(void (*isrptr) (void));
//void detachCNInterrupt(void);
//void activateCNInterrupt(uint8_t cnx, uint8_t pue);
//void deactivateCNInterrupt(uint8_t cnx);

//global variables

//empty interrupt handler
void empty_handler(void);

//clock switching
//Fvco=96..128Mhz (Volt range 2) or 96..344Mhz (Volt range 1)
//#define pll0x125()	pllSet( 1, 7)			//minimum pll. 2Mhz with HSI for the core
#define pll0x125()		pllSet( 2, 7)
#define pll0x25()		pllSet( 4, 7)
#define pll0x5()		pllSet( 8, 7)
#define pllx75()		pllSet(12, 7)
#define pll1x()			pllSet( 8, 3)
#define pll1x25()		pllSet(10, 3)
#define pll1x5()		pllSet(12, 3)
#define pll1x75()		pllSet(14, 3)
#define pll2x()			pllSet(16, 3)			//max frequency for hsi, per the datasheet
#define pll2x25()		pllSet(18, 3)
#define pll2x5()		pllSet(20, 3)			//max frequency for the core, tested at 80Mhz
#define pll2x75()		pllSet(22, 3)
#define pll3x()			pllSet(24, 3)
#define pll3x25()		pllSet(26, 3)
#define pll3x5()		pllSet(28, 3)
#define pll3x75()		pllSet(30, 3)
#define pll4x()			pllSet(16, 1)
#define pll4x25()		pllSet(17, 1)
#define pll4x5()		pllSet(18, 1)
#define pll4x75()		pllSet(19, 1)
#define pll5x()			pllSet(20, 1)
#define pll6x()			pllSet(24, 1)
#define pll6x5()		pllSet(26, 1)
#define pll7x()			pllSet(28, 1)
#define pll7x5()			pllSet(30, 1)
#define pll8x()			pllSet(32, 1)

#define SystemCoreClockHSI()				SystemCoreClockConfig(0b000)
#define SystemCoreClockHSIPLL()				SystemCoreClockHSIPLL4x()
#define SystemCoreClockHSIPLL2M()			SystemCoreClockHSIPLLx125()
#define SystemCoreClockHSIPLL4M()			SystemCoreClockHSIPLLx25()
#define SystemCoreClockHSIPLL8M()			SystemCoreClockHSIPLLx5()
#define SystemCoreClockHSIPLL16M()			SystemCoreClockHSIPLL1x()
#define SystemCoreClockHSIPLL24M()			SystemCoreClockHSIPLL1x5()
#define SystemCoreClockHSIPLL24M()			SystemCoreClockHSIPLL1x5()
#define SystemCoreClockHSIPLL32M()			SystemCoreClockHSIPLL2x()
#define SystemCoreClockHSIPLL40M()			SystemCoreClockHSIPLL2x5()
#define SystemCoreClockHSIPLL48M()			SystemCoreClockHSIPLL3x()			//highest working clock per datasheet
#define SystemCoreClockHSIPLL56M()			SystemCoreClockHSIPLL3x5()
#define SystemCoreClockHSIPLL64M()			SystemCoreClockHSIPLL4x()
#define SystemCoreClockHSIPLL72M()			SystemCoreClockHSIPLL4x5()
#define SystemCoreClockHSIPLL80M()			SystemCoreClockHSIPLL5x()
#define SystemCoreClockHSIPLL88M()			SystemCoreClockHSIPLL5x5()
#define SystemCoreClockHSIPLL96M()			SystemCoreClockHSIPLL6x()
#define SystemCoreClockHSIPLL104M()			SystemCoreClockHSIPLL6x5()
#define SystemCoreClockHSIPLL112M()			SystemCoreClockHSIPLL7x()			//highest working clock empirically
#define SystemCoreClockHSIPLL120M()			SystemCoreClockHSIPLL7x5()			//highest working clock empirically
//#define SystemCoreClockHSIPLL128M()			SystemCoreClockHSIPLL8x()			//doesn't work - mcu hangs
#define SystemCoreClockHSIPLLx125()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll0x125();SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLLx25()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll0x25();SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLLx5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll0x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL1x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll1x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL1x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll1x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL2x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll2x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL2x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll2x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL3x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll3x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL3x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll3x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL4x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll4x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL4x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll4x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL5x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll5x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL5x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll5x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL6x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll6x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL6x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll6x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL7x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll7x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL7x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll7x5(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSIPLL8x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI; pll8x();  SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSE()				SystemCoreClockConfig(0b001)
#define SystemCoreClockHSEPLL()				SystemCoreClockHSEPLL1x()
#define SystemCoreClockHSEPLL1x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; pll1x(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSEPLL1x5()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; pll1x5();SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSEPLL2x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; pll2x(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSEPLL3x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; pll3x(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockHSEPLL4x()			do {RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE; pll4x(); SystemCoreClockConfig(0b010);} while (0)
#define SystemCoreClockLSI()				SystemCoreClockConfig(0b011)
#define SystemCoreClockLSE()				SystemCoreClockConfig(0b100)

//mco output configuration
//mco: div 0..7 -> 1..128:1 - datasheet is wrong in suggesting that this is a 4-bit field. Code suggests that this is a 3-bit field
#define MCO2NOCLK(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (0 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2SYSCLK(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (1 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2HSI48(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (2 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2HSI16(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (3 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2HSE(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (4 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2PLLRCLK(div)					do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (5 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2LSI(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (6 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2LSE(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (7 << RCC_CFGR_MCOSEL_Pos);} while (0)
//below only available on G0B/G0C devices
#define MCO2PLLPCLK(div)					do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (8 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2PLLQCLK(div)					do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (9 << RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2RTCCLK(div)						do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (10<< RCC_CFGR_MCOSEL_Pos);} while (0)
#define MCO2RTCWAKEUP(div)					do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (11<< RCC_CFGR_MCOSEL_Pos);} while (0)

//clock
//switch system clock
//0b000->HSI
//0b001->HSE
//0b010->PLLRCLK
//0b011->LSI
//0b100->LSE
//need to confirm LSI/LSE
void SystemCoreClockConfig(uint8_t clk);
//set pll
//return Fvco: 96..344Mhz (volt scalling range 1)
//Fvco=Fpllin * N / (M + 1)
//N: 8..86, M = 0..7
void pllSet(uint8_t N, uint8_t M);
uint32_t AHBClock(void);					//ahb clock
uint32_t APBClock(void);					//APB clock
uint32_t F_PLLIN(void);						//PLL VCO input clock
//#define F_UART1		(((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (0<< 0))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (1<< 0))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (2<< 0))?F_HSI16:F_LSE)))
uint32_t u1FreqGet(void);					//uart1 clock rate

//global defines
#define IDCODE				(DBGMCU->IDCODE)
#define CPUID				(SCB->CPUID)
#define IDCODE				(DBGMCU->IDCODE)
#define CPUID				(SCB->CPUID)
//96bits/12bytes unique id
//usage: UID[0]..UID[11]
#define UID					((char *) 0x1FFF7590)

//global variables

//reset the mcu
void mcuInit(void);

//activate the isr handler
void systickAttachISR(void (*isrptr)(void));

//#define Mhz					000000ul	//suffix for Mhz
#define F_UART				(F_PCLK)	//8Mhz		//crystal frequency
#define UART_BR300			300ul		//buadrate=300
#define UART_BR600			600ul		//buadrate=600
#define UART_BR1200			1200ul		//buadrate=1200
#define UART_BR2400			2400ul		//buadrate=2400
#define UART_BR4800			4800ul		//buadrate=4800
#define UART_BR9600			9600ul		//buadrate=9600
#define UART_BR14400		14400ul		//buadrate=19200
#define UART_BR19200		19200ul		//buadrate=19200
#define UART_BR38400		38400ul		//buadrate=38400
#define UART_BR43000		43000ul		//buadrate=38400
#define UART_BR57600		57600ul		//buadrate=57600
#define UART_BR76800		76800ul		//buadrate=57600
#define UART_BR115200		115200ul	//buadrate=115200
#define UART_BR128000		128000ul	//buadrate=115200
#define UART_BR230400		230400ul	//buadrate=115200
#define UART_BR256000		256000ul	//buadrate=115200
#define UART_BR460800		460800ul	//buadrate=115200
#define UART_BR512000		512000ul	//buadrate=115200
#define UART_BR576000		576000ul	//buadrate=115200
#define UART_BR921600		921600ul	//buadrate=115200
#define UART_BR1000000		1000000ul	//buadrate=115200
#define UART_BR1250000		1250000ul	//buadrate=115200
#define UART_BR1500000		1500000ul	//buadrate=115200
#define UART_BR2000000		2000000ul	//buadrate=115200
//short notations
#define UART_BR0K3			UART_BR300
#define UART_BR0K6			UART_BR600
#define UART_BR1K2			UART_BR1200
#define UART_BR2K4			UART_BR2400
#define UART_BR4K8			UART_BR4800
#define UART_BR9K6			UART_BR9600
#define UART_BR14K4			UART_BR14400
#define UART_BR19K2			UART_BR19200
#define UART_BR38K4			UART_BR38400
#define UART_BR43K			UART_BR43000
#define UART_BR57K6			UART_BR57600
#define UART_BR76K8			UART_BR76800
#define UART_BR115K2		UART_BR115200
#define UART_BR128K			UART_BR128000
#define UART_BR256K			UART_BR256000
#define UART_BR460K8		UART_BR460800
#define UART_BR512K			UART_BR512000
#define UART_BR576K			UART_BR576000
#define UART_BR921K6		UART_BR921600
#define UART_BR1M			UART_BR1000000
#define UART_BR1M25			UART_BR1250000
#define UART_BR1M5			UART_BR1500000
#define UART_BR2M			UART_BR2000000
//for compatability
#define uart1Put(ch)		uart1Putch(ch)
#define uart1Get()			uart1Getch()

//initiate the hardware usart
void uart1Init(unsigned long baud_rate);
void uart1Putch(char ch);
void uart1Puts(char *str);
uint16_t uart1Available(void);
uint16_t uart1Busy(void);

void uart1Putline(char *ln);
//read a char from usart
uint8_t uart1Getch(void);
void u1Print(char *str, int32_t dat);
#define u1Println()			uart1Puts("\r\n")
#define uart1bps()			(F_UART1 * ((USART1->CR1 & (1<<15))?2:1) / USART1->BRR)
#define u1bps()				uart1bps()

//for compatability
#define uart2Put(ch)		uart2Putch(ch)
#define uart2Get()			uart2Getch()

//initiate the hardware usart
void uart2Init(unsigned long baud_rate);
void uart2Putch(char ch);
void uart2Puts(char *str);
uint16_t uart2Available(void);
uint16_t uart2Busy(void);
void uart2Putline(char *ln);

//read a char from usart
uint8_t uart2Getch(void);
void u2Print(char *str, int32_t dat);
#define u2Println()			uart2Puts("\r\n")
#define uart2bps()			(F_UART2 * ((USART2->CR1 & (1<<15))?2:1) / USART2->BRR)
#define u2bps()				uart2bps()
//end Serial

//lpuart1
//read a char from usart
//for compatability
#define lpuart1Put(ch)		lpuart1Putch(ch)
#define lpuart1Get()		lpuart1Getch()

//initiate the hardware usart
void lpuart1Init(unsigned long baud_rate);
void lpuart1Putch(char ch);
void lpuart1Puts(char *str);
uint16_t lpuart1Available(void);
uint16_t lpuart1Busy(void);
void lpuart1Putline(char *ln);

//read a char from usart
uint8_t lpuart1Getch(void);
void lpu1Print(char *str, int32_t dat);
#define lpu1Println()		lpuart1Puts("\r\n")
#define lpuart1bps()		(F_LPUART1 * ((LPUART1->CR1 & (1<<15))?2:1) * 16 / LPUART1->BRR * 16)
#define lpu1bps()			lpuart1bps()
//end Serial//end lpuart1

//timers
#define tmrCC1Get(tmr)		(tmr->CCR1)
#define tmrCC2Get(tmr)		(tmr->CCR2)
#define tmrCC3Get(tmr)		(tmr->CCR3)
#define tmrCC4Get(tmr)		(tmr->CCR4)
//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr1Get()			(TIM1->CNT)
#define tmr1SetPS(ps)		do {TIM1->PSC = (ps) - 1; TIM1->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr1GetPS()			(TIM1->PSC + 1l)
#define tmr1SetPR(pr)		TIM1->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr1GetPR()			(TIM1->ARR + 1l)
void tmr1OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
//output cmpare
void tmr1OC1SetPR(uint16_t pr);					//set period
void tmr1OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC2SetPR(uint16_t pr);					//set period
void tmr1OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC3SetPR(uint16_t pr);					//set period
void tmr1OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC4SetPR(uint16_t pr);					//set period
void tmr1OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr1IC1Init(uint8_t ps);								//activate input capture
#define tmr1CC1Get()		tmrCC1GET(TIM1)			//read tmr1 cc1 capture / compare value
void tmr1IC2Init(uint8_t ps);								//activate input capture
#define tmr1CC2Get()		tmrCC2GET(TIM1)			//read tmr1 cc1 capture / compare value
void tmr1IC3Init(uint8_t ps);								//activate input capture
#define tmr1CC3Get()		tmrCC3GET(TIM1)			//read tmr1 cc1 capture / compare value
void tmr1IC4Init(uint8_t ps);								//activate input capture
#define tmr1CC4Get()		tmrCC4GET(TIM1)			//read tmr1 cc1 capture / compare value

//initialize the timer3 (16bit)
void tmr2Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr2Get()			(TIM2->CNT)
#define tmr2SetPS(ps)		do {TIM2->PSC = (ps) - 1; TIM2->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr2GetPS()			(TIM2->PSC + 1l)
#define tmr2SetPR(pr)		TIM2->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr2GetPR()			(TIM2->ARR + 1l)
void tmr2OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
//output cmpare
void tmr2OC1SetPR(uint32_t pr);					//set period
void tmr2OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC2SetPR(uint32_t pr);					//set period
void tmr2OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC3SetPR(uint32_t pr);					//set period
void tmr2OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC4SetPR(uint32_t pr);					//set period
void tmr2OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr2IC1Init(uint8_t ps);								//activate input capture
#define tmr2CC1Get()		tmrCC1GET(TIM2)			//read tmr1 cc1 capture / compare value
void tmr2IC2Init(uint8_t ps);								//activate input capture
#define tmr2CC2Get()		tmrCC2GET(TIM2)			//read tmr1 cc1 capture / compare value
void tmr2IC3Init(uint8_t ps);								//activate input capture
#define tmr2CC3Get()		tmrCC3GET(TIM2)			//read tmr1 cc1 capture / compare value
void tmr2IC4Init(uint8_t ps);								//activate input capture
#define tmr2CC4Get()		tmrCC4GET(TIM2)			//read tmr1 cc1 capture / compare value

//initialize the timer3 (16bit)
void tmr3Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr3Get()			(TIM3->CNT)
#define tmr3SetPS(ps)		do {TIM3->PSC = (ps) - 1; TIM3->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr3GetPS()			(TIM3->PSC + 1l)
#define tmr3SetPR(pr)		TIM3->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr3GetPR()			(TIM3->ARR + 1l)
void tmr3OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
//output cmpare
void tmr3OC1SetPR(uint16_t pr);					//set period
void tmr3OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC2SetPR(uint16_t pr);					//set period
void tmr3OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC3SetPR(uint16_t pr);					//set period
void tmr3OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC4SetPR(uint16_t pr);					//set period
void tmr3OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr3IC1Init(uint8_t ps);								//activate input capture
#define tmr3CC1Get()		tmrCC1GET(TIM3)			//read tmr1 cc1 capture / compare value
void tmr3IC2Init(uint8_t ps);								//activate input capture
#define tmr3CC2Get()		tmrCC2GET(TIM3)			//read tmr1 cc1 capture / compare value
void tmr3IC3Init(uint8_t ps);								//activate input capture
#define tmr3CC3Get()		tmrCC3GET(TIM3)			//read tmr1 cc1 capture / compare value
void tmr3IC4Init(uint8_t ps);								//activate input capture
#define tmr3CC4Get()		tmrCC4GET(TIM3)			//read tmr1 cc1 capture / compare value

//initialize the timer4 (16bit)
void tmr4Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr4Get()			(TIM4->CNT)
#define tmr4SetPS(ps)		do {TIM4->PSC = (ps) - 1; TIM4->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr4GetPS()			(TIM4->PSC + 1l)
#define tmr4SetPR(pr)		TIM4->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr4GetPR()			(TIM4->ARR + 1l)
void tmr4OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
//output cmpare
void tmr4OC1SetPR(uint16_t pr);					//set period
void tmr4OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr4OC2SetPR(uint16_t pr);					//set period
void tmr4OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr4OC3SetPR(uint16_t pr);					//set period
void tmr4OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr4OC4SetPR(uint16_t pr);					//set period
void tmr4OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr4IC1Init(uint8_t ps);								//activate input capture
#define tmr4CC1Get()		tmrCC1GET(TIM4)			//read tmr1 cc1 capture / compare value
void tmr4IC2Init(uint8_t ps);								//activate input capture
#define tmr4CC2Get()		tmrCC2GET(TIM4)			//read tmr1 cc1 capture / compare value
void tmr4IC3Init(uint8_t ps);								//activate input capture
#define tmr4CC3Get()		tmrCC3GET(TIM4)			//read tmr1 cc1 capture / compare value
void tmr4IC4Init(uint8_t ps);								//activate input capture
#define tmr4CC4Get()		tmrCC4GET(TIM4)			//read tmr1 cc1 capture / compare value

//initialize the timer6 (16bit)
void tmr6Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr6Get()			(TIM6->CNT)
#define tmr6SetPS(ps)		do {TIM6->PSC = (ps) - 1; TIM6->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr6GetPS()			(TIM6->PSC + 1l)
#define tmr6SetPR(pr)		TIM6->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr6GetPR()			(TIM6->ARR + 1l)
void tmr6OVFAttachISR(void (*isrptr)(void));		//activate the isr handler

//initialize the timer7 (16bit)
void tmr7Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr7Get()			(TIM7->CNT)
#define tmr7SetPS(ps)		do {TIM7->PSC = (ps) - 1; TIM7->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr7GetPS()			(TIM7->PSC + 1l)
#define tmr7SetPR(pr)		TIM7->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr7GetPR()			(TIM7->ARR + 1l)
void tmr7OVFAttachISR(void (*isrptr)(void));		//activate the isr handler

//initialize the timer14 (16bit)
void tmr14Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr14Get()			(TIM14->CNT)
#define tmr14SetPS(ps)		do {TIM14->PSC = (ps) - 1; TIM14->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr14GetPS()		(TIM14->PSC + 1l)
#define tmr14SetPR(pr)		TIM14->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr14GetPR()		(TIM14->ARR + 1l)
void tmr14OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output compare
void tmr14OC1SetPR(uint16_t pr);					//set period
void tmr14OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr14OC2SetPR(uint16_t pr);					//set period
void tmr14OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr14IC1Init(uint8_t ps);								//activate input capture
#define tmr14CC1Get()		tmrCC1GET(TIM14)			//read tmr1 cc1 capture / compare value
void tmr14IC2Init(uint8_t ps);								//activate input capture
#define tmr14CC2Get()		tmrCC2GET(TIM14)			//read tmr1 cc1 capture / compare value

//initialize the timer14 (16bit)
void tmr15Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr15Get()			(TIM15->CNT)
#define tmr15SetPS(ps)		do {TIM15->PSC = (ps) - 1; TIM15->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr15GetPS()		(TIM15->PSC + 1l)
#define tmr15SetPR(pr)		TIM15->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr15GetPR()		(TIM15->ARR + 1l)
void tmr15OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output cmpare
void tmr155OC1SetPR(uint16_t pr);					//set period
void tmr155OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr155OC2SetPR(uint16_t pr);					//set period
void tmr155OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr15IC1Init(uint8_t ps);								//activate input capture
#define tmr15CC1Get()		tmrCC1GET(TIM15)			//read tmr1 cc1 capture / compare value
void tmr15IC2Init(uint8_t ps);								//activate input capture
#define tmr15CC2Get()		tmrCC2GET(TIM15)			//read tmr1 cc1 capture / compare value

//initialize the timer16 (16bit)
void tmr16Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr16Get()			(TIM16->CNT)
#define tmr16SetPS(ps)		do {TIM16->PSC = (ps) - 1; TIM16->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr16GetPS()		(TIM16->PSC + 1l)
#define tmr16SetPR(pr)		TIM16->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr16GetPR()		(TIM16->ARR + 1l)
void tmr16OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output cmpare
void tmr16OC1SetPR(uint16_t pr);					//set period
void tmr16OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr16OC2SetPR(uint16_t pr);					//set period
void tmr16OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr16IC1Init(uint8_t ps);								//activate input capture
#define tmr16CC1Get()		tmrCC1GET(TIM16)			//read tmr1 cc1 capture / compare value
void tmr16IC2Init(uint8_t ps);								//activate input capture
#define tmr16CC2Get()		tmrCC2GET(TIM16)			//read tmr1 cc1 capture / compare value

//initialize the timer17 (16bit)
#define tmr17Get()			(TIM17->CNT)
void tmr17Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr17SetPS(ps)		do {TIM17->PSC = (ps) - 1; TIM17->EGR |= TIM_EGR_UG;} while (0)	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr17GetPS()		(TIM17->PSC + 1l)
#define tmr17SetPR(pr)		TIM17->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period
#define tmr17GetPR()		(TIM17->ARR + 1l)
void tmr17OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output cmpare
void tmr17OC1SetPR(uint16_t pr);					//set period
void tmr17OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr17OC2SetPR(uint16_t pr);					//set period
void tmr17OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr17IC1Init(uint8_t ps);								//activate input capture
#define tmr17CC1Get()		tmrCC1GET(TIM17)			//read tmr1 cc1 capture / compare value
void tmr17IC2Init(uint8_t ps);								//activate input capture
#define tmr17CC2Get()		tmrCC2GET(TIM17)			//read tmr1 cc1 capture / compare value

//initialize tim21 as a synchronized 32-bit timer
//tim2 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr21Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr21SetPS(ps)		tmr2SetPS(ps)
#define tmr21GetPS()		tmr2GetPS()
uint64_t tmr21Get(void);				//read 32-bit timer value
uint64_t tmr21SetPR(uint64_t pr);		//set 32-bit timer period
uint64_t tmr21GetPR(void);				//read 32-bit timer period
#define tmr21OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

//initialize tim31 as a synchronized 32-bit timer
//tim3 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr31Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr31SetPS(ps)		tmr3SetPS(ps)
#define tmr31GetPS()		tmr3GetPS()
uint32_t tmr31Get(void);				//read 32-bit timer value
uint32_t tmr31SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr31GetPR(void);				//read 32-bit timer period
#define tmr31OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

void tmr151Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr151SetPS(ps)		tmr15SetPS(ps)
#define tmr151GetPS()		tmr15GetPS()
uint32_t tmr151Get(void);				//read 32-bit timer value
uint32_t tmr151SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr151GetPR(void);				//read 32-bit timer period
#define tmr151OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

void tmr171Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr1713SetPS(ps)	tmr17SetPS(ps)
#define tmr171GetPS()		tmr17GetPS()
uint32_t tmr171Get(void);				//read 32-bit timer value
uint32_t tmr171SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr171GetPR(void);				//read 32-bit timer period
#define tmr171OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

void tmr13Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr13SetPS(ps)		tmr1SetPS(ps)
#define tmr13GetPS()		tmr1GetPS()
uint32_t tmr13Get(void);				//read 32-bit timer value
uint32_t tmr13SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr13GetPR(void);				//read 32-bit timer period
#define tmr13OVFAttachISR(isrptr)		tmr3OVFAttachISR(isrptr)		//activate the isr handler

void tmr143Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr143SetPS(ps)		tmr14SetPS(ps)
#define tmr143GetPS()		tmr14GetPS()
uint32_t tmr143Get(void);				//read 32-bit timer value
uint32_t tmr143SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr143GetPR(void);				//read 32-bit timer period
#define tmr143OVFAttachISR(isrptr)		tmr3OVFAttachISR(isrptr)		//activate the isr handler

//lptim
#define LPTIM_PS1x			0			//prescaler 1:1
#define LPTIM_PS2x			1			//prescaler 2:1
#define LPTIM_PS4x			2			//prescaler 4:1
#define LPTIM_PS8x			3			//prescaler 8:1
#define LPTIM_PS16x			4			//prescaler 16:1
#define LPTIM_PS32x			5			//prescaler 32:1
#define LPTIM_PS64x			6			//prescaler 64:1
#define LPTIM_PS128x		7			//prescaler 128:1

void lptmr1Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define lptmr1Get()		(LPTIM1->CNT)
#define lptmr1Start()	LPTIM1->CR |= LPTIM_CR_SNGSTRT
#define lptmr1OVF()		(LPTIM1->ISR & LPTIM_ISR_ARRM)
#define lptmr1OVFCLR()	LPTIM1->ICR = LPTIM_ICR_ARRMCF	//write 1 to clear
#define lptmr1SetPR(pr)	LPTIM1->ARR = (pr) - 1
#define lptmr1GetPR()	(LPTIM1->ARR + 1ul)
#define lptmr1SetPS(ps)	LPTIM1->CFGR = (LPTIM1->CFGR &~LPTIM_CFGR_PRESC) | (((ps) << LPTIM_CFGR_PRESC_Pos) & LPTIM_CFGR_PRESC)
#define lptmr1GetPS()	((LPTIM1->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos)
void lptmr1AttachISR(void (*isr_ptr)(void));	//activate the isr handler

void lptmr2Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define lptmr2Get()		(LPTIM2->CNT)
#define lptmr2Start()	LPTIM2->CR |= LPTIM_CR_SNGSTRT
#define lptmr2OVF()		(LPTIM2->ISR & LPTIM_ISR_ARRM)
#define lptmr2OVFCLR()	LPTIM2->ICR = LPTIM_ICR_ARRMCF	//write 1 to clear
#define lptmr2SetPR(pr)	LPTIM2->ARR = (pr) - 1
#define lptmr2GetPR()	(LPTIM2->ARR + 1ul)
#define lptmr2SetPS(ps)	LPTIM2->CFGR = (LPTIM1->CFGR &~LPTIM_CFGR_PRESC) | (((ps) << LPTIM_CFGR_PRESC_Pos) & LPTIM_CFGR_PRESC)
#define lptmr2GetPS()	((LPTIM2->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos)
void lptmr2AttachISR(void (*isr_ptr)(void));	//activate the isr handler

//end lptim1/2
//pwm / oc
//global defines

//global variables

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm1Init(uint16_t TxCCP_PS);
#define pwm1GetPS()				tmr1GetPS()
#define pwm1SetPS(ps)			tmr1SetPS(ps)
#define pwm1GetPR()				tmr1GetPR()
#define pwm1SetPR(pr)			tmr1SetPR(pr)
//set duty cycle
#define pwm1SetDC1(dc)			TIM1->CCR1 = (dc)
#define pwm1SetDC2(dc)			TIM1->CCR2 = (dc)
#define pwm1SetDC3(dc)			TIM1->CCR3 = (dc)
#define pwm1SetDC4(dc)			TIM1->CCR4 = (dc)
#define pwm1GetDC1()			(TIM1->CCR1)
#define pwm1GetDC2()			(TIM1->CCR2)
#define pwm1GetDC3()			(TIM1->CCR3)
#define pwm1GetDC4()			(TIM1->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm2Init(uint16_t TxCCP_PS);
#define pwm2GetPS()				tmr2GetPS()
#define pwm2SetPS(ps)			tmr2SetPS(ps)
#define pwm2GetPR()				tmr2GetPR()
#define pwm2SetPR(pr)			tmr2SetPR(pr)
//set duty cycle
#define pwm2SetDC1(dc)			TIM2->CCR1 = (dc)
#define pwm2SetDC2(dc)			TIM2->CCR2 = (dc)
#define pwm2SetDC3(dc)			TIM2->CCR3 = (dc)
#define pwm2SetDC4(dc)			TIM2->CCR4 = (dc)
#define pwm2GetDC1()			(TIM2->CCR1)
#define pwm2GetDC2()			(TIM2->CCR2)
#define pwm2GetDC3()			(TIM2->CCR3)
#define pwm2GetDC4()			(TIM2->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm3Init(uint16_t TxCCP_PS);
#define pwm3GetPS()				tmr3GetPS()
#define pwm3SetPS(ps)			tmr3SetPS(ps)
#define pwm3GetPR()				tmr3GetPR()
#define pwm3SetPR(pr)			tmr3SetPR(pr)
//set duty cycle
#define pwm3SetDC1(dc)			TIM3->CCR1 = (dc)
#define pwm3SetDC2(dc)			TIM3->CCR2 = (dc)
#define pwm3SetDC3(dc)			TIM3->CCR3 = (dc)
#define pwm3SetDC4(dc)			TIM3->CCR4 = (dc)
#define pwm3GetDC1()			(TIM3->CCR1)
#define pwm3GetDC2()			(TIM3->CCR2)
#define pwm3GetDC3()			(TIM3->CCR3)
#define pwm3GetDC4()			(TIM3->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm14Init(uint16_t TxCCP_PS);
#define pwm14GetPS()			tmr1GetPS()
#define pwm14SetPS(ps)			tmr14SetPS(ps)
#define pwm14GetPR()			tmr14GetPR()
#define pwm14SetPR(pr)			tmr14SetPR(pr)
//set duty cycle
#define pwm14SetDC1(dc)			TIM14->CCR1 = (dc)
#define pwm14SetDC2(dc)			TIM14->CCR2 = (dc)
#define pwm14SetDC3(dc)			TIM14->CCR3 = (dc)
#define pwm14SetDC4(dc)			TIM14->CCR4 = (dc)
#define pwm14GetDC1()			(TIM14->CCR1)
#define pwm14GetDC2()			(TIM14->CCR2)
#define pwm14GetDC3()			(TIM14->CCR3)
#define pwm14GetDC4()			(TIM14->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS);
#define pwm15GetPS()			tmr15GetPS()
#define pwm15SetPS(ps)			tmr15SetPS(ps)
#define pwm15GetPR()			tmr15GetPR()
#define pwm15SetPR(pr)			tmr15SetPR(pr)
//set duty cycle
#define pwm15SetDC1(dc)			TIM15->CCR1 = (dc)
#define pwm15SetDC2(dc)			TIM15->CCR2 = (dc)
#define pwm15SetDC3(dc)			TIM15->CCR3 = (dc)
#define pwm15SetDC4(dc)			TIM15->CCR4 = (dc)
#define pwm15GetDC1()			(TIM15->CCR1)
#define pwm15GetDC2()			(TIM15->CCR2)
#define pwm15GetDC3()			(TIM15->CCR3)
#define pwm15GetDC4()			(TIM15->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm16Init(uint16_t TxCCP_PS);
#define pwm16GetPS()			tmr16GetPS()
#define pwm16SetPS(ps)			tmr16SetPS(ps)
#define pwm16GetPR()			tmr16GetPR()
#define pwm16SetPR(pr)			tmr16SetPR(pr)
//set duty cycle
#define pwm16SetDC1(dc)			TIM16->CCR1 = (dc)
#define pwm16SetDC2(dc)			TIM16->CCR2 = (dc)
#define pwm16SetDC3(dc)			TIM16->CCR3 = (dc)
#define pwm16SetDC4(dc)			TIM16->CCR4 = (dc)
#define pwm16GetDC1()			(TIM16->CCR1)
#define pwm16GetDC2()			(TIM16->CCR2)
#define pwm16GetDC3()			(TIM16->CCR3)
#define pwm16GetDC4()			(TIM16->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm17Init(uint16_t TxCCP_PS);
#define pwm17GetPS()			tmr17GetPS()
#define pwm17SetPS(ps)			tmr17SetPS(ps)
#define pwm17GetPR()			tmr17GetPR()
#define pwm17SetPR(pr)			tmr17SetPR(pr)
//set duty cycle
#define pwm17SetDC1(dc)			TIM17->CCR1 = (dc)
#define pwm17SetDC2(dc)			TIM17->CCR2 = (dc)
#define pwm17SetDC3(dc)			TIM17->CCR3 = (dc)
#define pwm17SetDC4(dc)			TIM17->CCR4 = (dc)
#define pwm17GetDC1()			(TIM17->CCR1)
#define pwm17GetDC2()			(TIM17->CCR2)
#define pwm17GetDC3()			(TIM17->CCR3)
#define pwm17GetDC4()			(TIM17->CCR4)

//adc
//adc channel definitions
#define ADC_CH0			(1ul<<0)
#define ADC_CH1			(1ul<<1)
#define ADC_CH2			(1ul<<2)
#define ADC_CH3			(1ul<<3)
#define ADC_CH4			(1ul<<4)
#define ADC_CH5			(1ul<<5)
#define ADC_CH6			(1ul<<6)
#define ADC_CH7			(1ul<<7)
#define ADC_CH8			(1ul<<8)
#define ADC_CH9			(1ul<<9)
#define ADC_CH10		(1ul<<10)
#define ADC_CH11		(1ul<<11)
#define ADC_CH12		(1ul<<12)
#define ADC_CH13		(1ul<<13)
#define ADC_CH14		(1ul<<14)
#define ADC_CH15		(1ul<<15)
#define ADC_CH16		(1ul<<16)
#define ADC_CH17		(1ul<<17)
#define ADC_TS			ADC_CH12				//tempeature sensor on CH12
#define ADC_VREFINT		ADC_CH13				//internal reference on CH13 @ 1.20v
#define ADC_BAT3		ADC_CH14				//Vbat / 3
#define ADC_VREF		ADC_VREFINT				//internal reference
#define ADC_TEMP		ADC_TS					//temperature sensor
#define ADC_ARAND		ADC_VREFINT				//the channel for random number generators
#define TSCAL1			((*(int32_t *) 0x1fff75a8) & 0x0fff)		//~1041 @30c, VDDA = 3.0v
#define TSCAL2			((*(int32_t *) 0x1fff75ca) & 0x0fff)		//not present on G030
#define VREFCAL			((*(int32_t *) 0x1fff75aa) & 0x0fff)		//not present on G030

//convert temperature sensor adc reading into temperaturex10
//follow the datasheet. 3.3v Vref (3.0v for my board), 12bit adc
//convert temperature sensor adc reading into temperaturex10
//two alternatives to calculate temperature. both work
//#define ADC2Cx100(adc)		((8500 - 3000) * ((adc) - TSCAL1) / (TSCAL2 - TSCAL1) + 3000)	//per reference manual - but TSCAL2 not available on G030
#define ADC2Cx100(adc)	((760 - ADC2mv(adc)) * 10 * 100 / (25) + 3000)					//per datasheet, using average slope. 
#define ADC2Fx100(adc)	(ADC2Cx100(adc) * 9 / 5 + 3200)
#define VREFadc			1585l					//VREF ADC reading
#define VREFmv			1200l					//Vref voltage in mv
#define VDDAmv			(VREFmv * 4096 / VREFadc)	//VADD=3.3v
#define ADC2mv(adc)		((adc) * VDDAmv / 4096)	//convert 12-bit adc to mv

//rest the adc
//automatic sampling (ASAM=1), manual conversion
void adc1Init(void);
uint16_t adc1Read(uint32_t adc_ch);

//read the adc
#define analogRead(ch)	adc1Read(ch)
//end ADC

//input capture
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1Init(void);
void ic1AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic1Get(void);							//read buffer value

void ic2Init(void);
void ic2AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic2Get(void);							//read buffer value

void ic3Init(void);
void ic3AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic3Get(void);							//read buffer value

void ic4Init(void);
void ic4AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic4Get(void);							//read buffer value

void ic5Init(void);
void ic5AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic5Get(void);							//read buffer value
//end input capture

//int
void int0Init(void);							//initialize the module
void int0AttachISR(void (*isrptr) (void));		//attach user isr

void int1Init(void);							//initialize the module
void int1AttachISR(void (*isrptr) (void));		//attach user isr

void int2Init(void);							//initialize the module
void int2AttachISR(void (*isrptr) (void));		//attach user isr

void int3Init(void);							//initialize the module
void int3AttachISR(void (*isrptr) (void));		//attach user isr

void int4Init(void);							//initialize the module
void int4AttachISR(void (*isrptr) (void));		//attach user isr
//end int

//spi
void spi1Init(uint32_t br);							//reset the spi
#define spi1Busy()			(SPI1->SR & SPI_SR_BSY)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi1Avail()			(SPI1->SR & SPI_SR_RXNE)	//receive buffer not empty -> there is data
#define spi1Wait()			while (spi1Busy()) continue
#define spi1Write(dat)		do {spi1Wait(); SPI1->DR = (dat);} while (0)
#define spi1Read()			(SPI1->DR)				//read from the buffer

void spi2Init(uint32_t br);							//reset the spi
#define spi2Busy()			(SPI2->SR & SPI_SR_BSY)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi2Avail()			(SPI2->SR & SPI_SR_RXNE)	//receive buffer not empty -> there is data
#define spi2Wait()			while (spi2Busy()) continue
#define spi2Write(dat)		do {spi2Wait(); SPI2->DR = (dat);} while (0)
#define spi2Read()			(SPI2->DR)				//read from the buffer
//end spi

//i2c
//i2c1
void i2c1Init(uint32_t br);							//i2c1 initialization

//i2c2
void i2c2Init(uint32_t br);							//i2c1 initialization
//end i2c

//rtcc
#define RCC_BDCR_RTCSEL_NOCLOCK			(0<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSE				(1<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSI				(2<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_HSE				(3<<RCC_BDCR_RTCSEL_Pos)	//hse/32
//bcd to dec conversion
#define BCD2DEC(bcd)				((((bcd) & 0xf0)>>4)*10+((bcd) & 0x0f))		//convert bcd to int
#define DEC2BCD(dec)				(((((dec) / 10)<<4) +((dec) % 10)) & 0xff)	//convert int to bcd
//initialize the rtc
//prep: see RTC_PREP in the datasheet
//rtc_clk: RCC_BDCR_RTCSEL_LSI, RCC_BDCR_RTCSEL_LSE and RCC_BDCR_RTCSEL_HSE
//prep consists of a 7-bit prediv_a (msb) and 15-bit prediv_b (lsb)
void RTCInit(uint32_t rtc_clk, uint32_t prep);					//initialize rtc
#define RTCInitLSI()				RTCInit(RCC_BDCR_RTCSEL_LSI, F_LSI)
#define RTCInitLSE()				RTCInit(RCC_BDCR_RTCSEL_LSE, F_LSE)
#define RTCInitHSE()				RTCInit(RCC_BDCR_RTCSEL_HSE, F_HSE/32)
void RTCSetTime(uint32_t val);									//set rtc time
#define RTCGetTime()				(RTC->TR)					//get rtc time
#define RTCGetSec()					BCD2DEC(((RTCGetTime() >> 0) & 0x7f))	//get rtc sec
#define RTCGetMin()					BCD2DEC(((RTCGetTime() >> 8) & 0x7f))	//get rtc min
#define RTCGetHour()				BCD2DEC(((RTCGetTime() >>16) & 0x3f))	//get rtc hour
void RTCSetDate(uint32_t val);									//set rtc date
#define RTCGetDate()				(RTC->DR)					//get rtc date
#define RTCGetDay()					BCD2DEC(((RTCGetDate() >> 0) & 0x3f))	//get rtc day
#define RTCGetMonth()				BCD2DEC(((RTCGetDate() >> 8) & 0x1f))	//get rtc month
#define RTCGetWDay()				BCD2DEC(((RTCGetDate() >>13) & 0x07))	//get rtc wday (1..7, 0 forbidden)
#define RTCGetYear()				BCD2DEC(((RTCGetDate() >>16) & 0xff))	//get rtc year
time_t RTC2time(time_t *t);			//convert rtcc to time_t
time_t time2RTC(time_t t);			//turn time (=time_t) to RTCC
int16_t RTCSetCal(int16_t cal);		//set calibration
int16_t RTCGetCal(void);			//set calibration
uint8_t RTCHalfsec(void);			//test for subsecond: 0->first half of a second, 1->second half of a second
//end rtcc

//software rtc

//global variables
typedef struct {
	time_t time;						//second counter since 1/1/1970
	uint32_t tick;						//tick counter at last update
	int16_t cal;						//rtc calibration, in ppm. +=faster, -=slower
	uint8_t halfsec;					//0=first half sec, 1=2nd half sec
} sRTC_TypeDef;

void sRTCInit(void);					//initialize software counter //calibration from -128ppm to +128ppm, if sRTC_RATE = 1M
void sRTCSetCal(int16_t cal);			//set sRTC calibration
void sRTCISR(void);						//increment -> called sRTC_CALLRATE times per second
#define sRTCUpdate()	sRTCISR()		//for compatability
uint32_t sRTCTick(void);				//read rtc tick
time_t sRTC2time(time_t *t);			//read sRTC second counter
time_t time2sRTC(time_t t);				//set sRTC second counter
uint8_t sRTCHalfsec(void);				//return 0: first half of a sec, 1: 2nd half of a sec
#define sRTCGetSec()	(sRTC2time(NULL) % 60)	//for compatability
//end software RTC

//extint
#define EXTI_FALLING	FALLING				//trigger on falling edge
#define EXTI_RISING		RISING				//trigger on rising edge
#define EXTI_BOTH		CHANGE				//trigger on falling and rising edges

//pin: one pin only
//gpio: GPIOA..GPIOG
//void extiInit(GPIO_TypeDef * gpio, uint16_t pin, uint8_t edge);
//void extiAttachISR(uint16_t pin, void (*isr_ptr)(void));		//install user handler
void extiInit(PIN_TypeDef pinx, uint8_t edge);
void extiAttachISR(PIN_TypeDef pinx, void (*isr_ptr)(void));		//install user handler
//end extint

//analog random number generator
//random number generators
#define arand1()			(analogRead(ADC_ARAND) & 0x01)	//generate 1-bit random number
uint8_t arand4(void);							//generate 4-bit random numbers
uint8_t arand8(void);							//generate 8-bit random numbers
uint16_t arand16(void);							//generate 16-bit random numbers
uint32_t arand32(void);							//generate 32-bit random numbers
//end analog random number generator

//circular buffer

//circular buffer to store char
typedef struct {
    cbuffer_t buffer[cBUFF_SIZE];		//buffer
    uint8_t writeIndex;					//index for the current write-> advance first and then write
    uint8_t readIndex;					//index for the current read -> advance first and then read
} CB_TypeDef;

void cbInit(CB_TypeDef *cbuffer);		//reset the cbuffer
char cbEmpty(CB_TypeDef *cbuffer);		//returns true if cbuffer is empty
char cbFull(CB_TypeDef *cbuffer);		//returns true if cbuffer is full;
cbuffer_t cbWrite(CB_TypeDef *cbuffer, cbuffer_t dat);	//write to the buffer - needs to test if the buffer is full first
cbuffer_t cbRead(CB_TypeDef *cbuffer);	//read from the buffer - needs to test if the buffer is empty
void str2cb(CB_TypeDef *cb, char *str);	//write string to CB
void cb2str(char *str, CB_TypeDef *cb);	//write CB to string
//end circular buffer

#if defined(RNG)
//true rng
void rngInit(void);						//initialize the rng
#define rngGet()						(RNG->DR)
#define rngReady()						(RNG->SR & RNG_SR_DRDY)
//end true rng
#endif	//rng

#if defined(AES)
//initalize the AES: ECB mode, polling only
//128 bit blocks
void aesInit(void);
//ciper a message
char *aesCipher(char *message);
#endif	//aes

//tamper backup registers
//unlock the domain
//on some chips, it requires a set of magic numbers
#define bkpUnlock()		do {PWR->CR1 |= PWR_CR1_DBP;} while (!(PWR->CR1 & PWR_CR1_DBP))	//1->enable write assess, 0->disable write assess
#define bkpLock()		do {PWR->CR1 &=~PWR_CR1_DBP;} while ( (PWR->CR1 & PWR_CR1_DBP))	//1->enable write assess, 0->disable write assess

//reset the tamper to use the backup registers
void bkpInit(void);


//read backup register: 0..4 on G030
#define bkp0Get(n)		(TAMP->BKP0R)			//read bkp0
#define bkp0Set(val)	do {bkpUnlock(); TAMP->BKP0R = (val); bkpLock();} while (0)		//write bkp0
#define bkp1Get(n)		(TAMP->BKP1R)			//read bkp1
#define bkp1Set(val)	do {bkpUnlock(); TAMP->BKP1R = (val); bkpLock();} while (0)		//write bkp1
#define bkp2Get(n)		(TAMP->BKP2R)			//read bkp2
#define bkp2Set(val)	do {bkpUnlock(); TAMP->BKP2R = (val); bkpLock();} while (0)		//write bkp2
#define bkp3Get(n)		(TAMP->BKP3R)			//read bkp3
#define bkp3Set(val)	do {bkpUnlock(); TAMP->BKP3R = (val); bkpLock();} while (0)		//write bkp3
#define bkp4Get(n)		(TAMP->BKP4R)			//read bkp4
#define bkp4Set(val)	do {bkpUnlock(); TAMP->BKP4R = (val); bkpLock();} while (0)		//write bkp4

#endif	//st32duino_h
