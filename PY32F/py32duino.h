#ifndef _PY32F0XX_H
#define _PY32F0XX_H

//PY32duino code
// - using PY32F03002/003/030 chip
// - free running systick for ticks
// - 
// - version history
// - v0.1, 07/02/2023: initial port from stm32f0
// - v0.2, 08/15/2023: implemented gpio, timer, and pwm functionalities
// - v0.2a,08/19/2023: added hardware and software RTC support
// v v0.3, 08/20/2023: implemented PLL
//
//
//               PY32F002AF/TSSOP20
//      GND     |=====================|    Vdd
//       |      |                     |     |
//       +------| GND            Vdd  |-----+
//              |                     |
//              |                PA14 |<----- SWCLK (Pin3)
//              |                     |
//       +------| BOOT0          PA13 |<----> SWDIO (Pin2)
//              |                     |
//              |                     |
//              |                 PA4 |--+--> GND
//              |                     |  |
//       +----->| OSCin/PF0       PA3 |--+--> LED
//    [Xtal]    |                     |
//       +-----<| OSCout/PF1          |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
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

#include <py32f0xx.h>						//we use PY32F0
#include <string.h>							//we use strcpy()
#include <stdint.h>							//we use uint types
#include <time.h>							//we use time routines

//hardware configuration
//#define USE_CORETICK						//use coretick/DWT for tick generation - ignored for STM32F0
#define uRAM_SIZE			40				//uart buffer size

//oscillator configuration
#define F_XTAL				10000000ul		//crystal frequency, user-specified
#define F_LSE				32768			//watch crystal frequency, user-specified

//configuration for circular buffer
/*******NOTE********************************/
/******cBUFF_SIZE: must be power of 2*******/
/*******************************************/
#define cBUFF_SIZE 			32				//must be power of 2: 0..256 -> for buffers with more than 256 elements, change data type for writeIndex/readIndex
typedef char cbuffer_t;						//storage unit for the buffer
//end configuration for circular buffer
//end user specification

//pin config
#define U1TX2PIN()					U1TX2PA2()			//PB6, PA2, PA9, PA14, PF1
//#define U1RX2PIN()					U1RX2PB7()			//PB7, PA3, PA10,PA13, PF0
#define U2TX2PIN()					U2TX2PA2()			//PA2, PA14
//#define U2RX2PIN()					U2RX2PA3()			//PA3, PA15

//pwm output pins
//pwm1 otput pins
#define TIM1CH1toPIN()				TIM1CH1toPA3()		//PA8, PA3
//#define TIM1CH2toPIN()				TIM1CH2toPA9()		//PA9, PA13
#define TIM1CH3toPIN()				TIM1CH3toPB6()		//PA0, PA10, PB6
#define TIM1CH4toPIN()				TIM1CH4toPA1()		//PA1
//#define TIM1CH1NtoPIN()				TIM1CH1NtoPA7()		//PA0, PA7, PB13
//#define TIM1CH2NtoPIN()				TIM1CH2NtoPB0()		//PA1, PB0
//#define TIM1CH3NtoPIN()				TIM1CH3NtoPB1()		//PB1

//pwm3 output pins
//#define TIM3CH1toPIN()				TIM3CH1toPA6()		//PA2, PA6, PB4
//#define TIM3CH2toPIN()				TIM3CH2toPA7()		//PA5, PA7, PB5
#define TIM3CH3toPIN()				TIM3CH3toPB0()		//PB0, PF3
#define TIM3CH4toPIN()				TIM3CH4toPB1()		//PB1

//pwm14 output pins
#define TIM14CH1toPIN()				TIM14CH1toPA4()		//PA4, PA7, PB1, PF0, PF1

//pwm15 output pins
//#define TIM15CH1toPIN()				TIM15CH1toPA2()		//PA2, PB14
//#define TIM15CH2toPIN()				TIM15CH2toPA3()		//PA3, PB15
//#define TIM15CH1NtoPIN()			TIM15CH1NtoPB15()	//PB15

//pwm16 output pins
#define TIM16CH1toPIN()				TIM16CH1toPA6()		//PA6, PB8
//#define TIM16CH1NtoPIN()			TIM16CH1NtoPB6()	//PB6

//pwm17 output pins
#define TIM17CH1toPIN()				TIM17CH1toPA7()		//PA7, PB8
//#define TIM17CH1NtoPIN()			TIM17CH1NtoPB7()	//PB7

//input capture pins

//spi pins
//#define NSS1toPIN()			NSS1toPA4()					//
//#define SCK1toPIN()			SCK1toPA5()					//
//#define MOSI1toPIN()		MOSI1toPA7()				//
//#define MISO1toPIN()		MISO1toPA6()				//
//end spi pins

//i2c pins
#define SCL1toPIN()			SCL1toPA9()					//PA3, PA8, PA9, PA10, PA11, PB6, PB8, PF1
#define SDA1toPIN()			SDA1toPA10()				//PA2, PA9, PA10,PA11, PA12, PB7, PB8, PF0

//mco pin
#define MCOtoPIN()			MCOtoPA5()					//PA1, PA5, PA8, PA9, PA13, PA14, PF2
//end pin configuration

#define F_SYSCLK			(SYSClock())
#define F_HCLK				(AHBClock())
#define F_PCLK				(APBClock())
#define F_CPU				(F_FCLK)					//peripheral block runs at F_PHB - default = F_CPU / 1
#define F_FCLK				F_HCLK
#define F_PLL				PLLClock()
#define F_PHB				(F_APB)						//cpu runs at 8Mhz/2 by default -> Fxtal = 8Mhz. *4 for PLL. RCDIV set to 0 (1:1 postscaler)
#define F_AHB				(F_HCLK)
#define F_APB				(F_PCLK)
#define F_HSE				F_XTAL
#define F_LSI				32768ul						//LSI frequency = 32768, in hz - measured to be 32600Hz
#define F_HSI				F_HSIGet()					//HSI frequency = 24Mhz, in hz - uncalibrated, nominal.
#define F_HSI24				24000000ul					//hsi frequency = 24Mhz
#define F_HSI22				22120000ul					//hsi frequency = 22.12Mhz
#define F_HSI16				16000000ul					//hsi frequency = 16Mhz
#define F_HSI8				8000000ul					//hsi frequency = 8Mhz
#define F_HSI4				4000000ul					//hsi frequency = 4Mhz
#define HSI_FS24			(4<<RCC_ICSCR_HSI_FS_Pos)	//0b100->24Mhz
#define HSI_FS22			(3<<RCC_ICSCR_HSI_FS_Pos)	//0b011->22.12Mhz
#define HSI_FS16			(2<<RCC_ICSCR_HSI_FS_Pos)	//0b010->16Mhz
#define HSI_FS8				(1<<RCC_ICSCR_HSI_FS_Pos)	//0b001->8Mhz
#define HSI_FS4				(0<<RCC_ICSCR_HSI_FS_Pos)	//0b000->4Mhz
#define HSI24TRIM			((*(uint32_t *) 0x1fff0f10) & 0x1fff)	//hsi trim for 24Mhz, [12..0] effective
#define HSI22TRIM			((*(uint32_t *) 0x1fff0f0c) & 0x1fff)	//hsi trim for 22.12Mhz
#define HSI16TRIM			((*(uint32_t *) 0x1fff0f08) & 0x1fff)	//hsi trim for 16Mhz
#define HSI8TRIM			((*(uint32_t *) 0x1fff0f04) & 0x1fff)	//hsi trim for 8Mhz
#define HSI4TRIM			((*(uint32_t *) 0x1fff0f00) & 0x1fff) 	//hsi trim for 4Mhz
#define LSITRIM				((*(uint32_t *) 0x1fff0fa4) & 0x01ff)	//lsi trim
#define F_HCLK8				(F_HCLK/8)
#define F_TIMPCLK			(((RCC->CFGR & RCC_CFGR_PPRE) == RCC_CFGR_PPRE_DIV1)?F_APB:(F_APB*2))
#define F_UART1				(u1FreqGet())
#define F_I2C				((RCC->CFGR3 & RCC_CFGR3_I2C1SW)?F_SYSCLK:F_HSI)
#define PWM_PR				0xffff						//pwm period - don't change
void SystemCoreClockConfig(uint32_t clk);				//change clock
//set hsi frequency
uint32_t F_HSISet(uint32_t f_hsi);						//hsi_fs = HSI_FS24, HSI_FS22, HSI_FS16, HSI_FS8, HSI_FS4
//get hsi frequency
uint32_t F_HSIGet(void);								//hsi_fs = HSI_FS24, HSI_FS22, HSI_FS16, HSI_FS8, HSI_FS4
uint32_t PLLClock(void);
uint32_t SYSClock(void);
uint32_t AHBClock(void);								//get AHB speed
uint32_t APBClock(void);								//get APB speed
uint32_t TIMClock(void);								//get TIM_PCLK speed
uint32_t u1FreqGet(void);								//uart1 speed = APBClock()
void pllSet(void);										//SET_BIT pll, fixed at 2x. Just set Flash latency at 1 (>24Mhz AHBClock)
#define SystemCoreClockHSI()		SystemCoreClockHSI8M()	//default to 8M
#define SystemCoreClockHSI4M()		do {F_HSISet(HSI_FS4); SystemCoreClockConfig(RCC_CFGR_SW_HSI);} while (0)
#define SystemCoreClockHSI8M()		do {F_HSISet(HSI_FS8); SystemCoreClockConfig(RCC_CFGR_SW_HSI);} while (0)
#define SystemCoreClockHSI16M()		do {F_HSISet(HSI_FS16);SystemCoreClockConfig(RCC_CFGR_SW_HSI);} while (0)
#define SystemCoreClockHSI22M()		do {F_HSISet(HSI_FS22);SystemCoreClockConfig(RCC_CFGR_SW_HSI);} while (0)
#define SystemCoreClockHSI24M()		do {F_HSISet(HSI_FS24);SystemCoreClockConfig(RCC_CFGR_SW_HSI);} while (0)
#define SystemCoreClockHSI44M()		SystemCoreClockHSI22Mx2()
#define SystemCoreClockHSI48M()		SystemCoreClockHSI24Mx2()
#define SystemCoreClockHSIPLL()		SystemCoreClockHSI24MPLL()
//PLL input frequency > 16Mhz. Otherwise it does not work
#define SystemCoreClockHSI4MPLL()	do {SystemCoreClockHSI4M();  RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockConfig(RCC_CFGR_SW_PLL); SystemCoreClockUpdate();} while (0)
#define SystemCoreClockHSI8MPLL()	do {SystemCoreClockHSI8M();  RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockConfig(RCC_CFGR_SW_PLL); SystemCoreClockUpdate();} while (0)
#define SystemCoreClockHSI16MPLL()	do {SystemCoreClockHSI16M(); RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockConfig(RCC_CFGR_SW_PLL); SystemCoreClockUpdate();} while (0)
//end PLL input frequency
#define SystemCoreClockHSI22MPLL()	do {SystemCoreClockHSI22M(); RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockConfig(RCC_CFGR_SW_PLL); SystemCoreClockUpdate();} while (0)
#define SystemCoreClockHSI24MPLL()	do {SystemCoreClockHSI24M(); RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockConfig(RCC_CFGR_SW_PLL); SystemCoreClockUpdate();} while (0)
#define SystemCoreClockHSI16Mx2()	SystemCoreClockHSI16MPLL()
#define SystemCoreClockHSI22Mx2()	SystemCoreClockHSI22MPLL()
#define SystemCoreClockHSI24Mx2()	SystemCoreClockHSI24MPLL()

#define SystemCoreClockHSE()		do {RCC->ECSCR = (RCC->ECSCR &~RCC_ECSCR_HSE_FREQ) | RCC_ECSCR_HSE_FREQx; SystemCoreClockConfig(RCC_CFGR_SW_HSE);} while (0)
//***two conditions: 1) Xtal > 16Mhz, and 2) Flash latency = 1 for AHBClock > 24Mhz***
#define SystemCoreClockHSEPLL()		do {SystemCoreClockHSE();    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC; pllSet(); SystemCoreClockUpdate();} while (0)
#define SystemCoreClockLSI()		do {SystemCoreClockConfig(RCC_CFGR_SW_LSI);} while (0)
#define SystemCoreClockLSE()		SystemCoreClockConfig(RCC_CFGR_SW_LSE)
uint32_t HSITrimGet(void);										//((RCC->ICSCR & RCC_ICSCR_HSI_TRIM) >> RCC_ICSCR_HSI_TRIM_Pos)
#define HSITrimSet(trim)			do {RCC->ICSCR = (RCC->ICSCR &~RCC_ICSCR_HSI_TRIM) | (((HSITrimGet() + (trim)) << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM);} while (0)
#define LSITrimGet()				(LSITRIM)					//((RCC->ICSCR & RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos)
#define LSITrimSet(trim)			do {RCC->ICSCR = (RCC->ICSCR &~RCC_ICSCR_LSI_TRIM) | (((LSITrimGet() + (trim)) << RCC_ICSCR_LSI_TRIM_Pos) & RCC_ICSCR_LSI_TRIM);} while (0)
//clock switching
#define RCC_CFGR_SW_HSI				(0ul<<RCC_CFGR_SW_Pos)		//HSISYS
#define RCC_CFGR_SW_HSE				(1ul<<RCC_CFGR_SW_Pos)		//HSE
#define RCC_CFGR_SW_PLL				(2ul<<RCC_CFGR_SW_Pos)		//PLL
#define RCC_CFGR_SW_LSI				(3ul<<RCC_CFGR_SW_Pos)		//LSI
#define RCC_CFGR_SW_LSE				(4ul<<RCC_CFGR_SW_Pos)		//LSE
#if F_XTAL > 16000000ul
#define RCC_ECSCR_HSE_FREQx			(3 << RCC_ECSCR_HSE_FREQ_Pos)	//0b11 for 16-24Mhz
#elif F_XTAL > 8000000ul
#define RCC_ECSCR_HSE_FREQx			(2 << RCC_ECSCR_HSE_FREQ_Pos)	//0b10 for 8-16Mhz
#elif F_XTAL > 4000000ul
#define RCC_ECSCR_HSE_FREQx			(1 << RCC_ECSCR_HSE_FREQ_Pos)	//0b01 for 4-8Mhz
#else
#define RCC_ECSCR_HSE_FREQx			(0 << RCC_ECSCR_HSE_FREQ_Pos)	//0b00 for hse off
#endif	//ecscr

//mco - div=0..7->1:1 -> 128:1
#define MCO2NOCLK(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (0<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2SYSCLK(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (1<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2HSI10M(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (2<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2HSI(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (3<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2HSE(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (4<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2PLL(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (5<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2LSI(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (6<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)
#define MCO2LSE(div)				do {RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOSEL) | (7<<RCC_CFGR_MCOSEL_Pos); RCC->CFGR = (RCC->CFGR &~RCC_CFGR_MCOPRE) | (((div) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE);} while (0)

	//pin config
//uart1/2/3 pin config
#define U1TX2PB6()			pinAFIO(PB6, AFIO0)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOB->OTYPER &=~(1<< 6); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 6%8)))) | (0<<(4*( 6%8)));} while (0)	//PB6, AF0
#define U1RX2PB7()			pinAFIO(PB7, AFIO0)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOB->OTYPER &=~(1<< 7); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 7%8)))) | (0<<(4*( 7%8)));} while (0)	//PB7, AF0
#define U1TX2PA2()			pinAFIO(PA2, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 2))) | (/*0b10*/0x02<<(2* 2)); GPIOA->OTYPER &=~(1<< 2); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 2))) | (/*0b01*/0x01<<(2* 2)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 2%8)))) | (1<<(4*( 2%8)));} while (0)	//PA2, AF1
#define U1RX2PA3()			pinAFIO(PA3, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 3))) | (/*0b10*/0x02<<(2* 3)); GPIOA->OTYPER &=~(1<< 3); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 3))) | (/*0b01*/0x01<<(2* 3)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 3%8)))) | (1<<(4*( 3%8)));} while (0)	//PA3, AF1
#define U1TX2PA9()			pinAFIO(PA9, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (1<<(4*( 9%8)));} while (0)	//PA9, AF1
#define U1RX2PA10()			pinAFIO(PA10,AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (1<<(4*(10%8)));} while (0)	//PA10, AF1
#define U1TX2PA14()			pinAFIO(PA14,AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*14))) | (/*0b10*/0x02<<(2*14)); GPIOA->OTYPER &=~(1<<14); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*14))) | (/*0b01*/0x01<<(2*14)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(14%8)))) | (1<<(4*(14%8)));} while (0)	//PA14, AF1
#define U1RX2PA13()			pinAFIO(PA8, AFIO8)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOA->OTYPER &=~(1<<15); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(15%8)))) | (1<<(4*(15%8)));} while (0)	//PA15, AF1
#define U1TX2PF1()			pinAFIO(PF1, AFIO8)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*14))) | (/*0b10*/0x02<<(2*14)); GPIOA->OTYPER &=~(1<<14); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*14))) | (/*0b01*/0x01<<(2*14)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(14%8)))) | (1<<(4*(14%8)));} while (0)	//PA14, AF1
#define U1RX2PF0()			pinAFIO(PF0, AFIO8)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOA->OTYPER &=~(1<<15); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(15%8)))) | (1<<(4*(15%8)));} while (0)	//PA15, AF1

//uart2 pins
#define U2TX2PA2()			pinAFIO(PA2, AFIO4)
#define U2RX2PA3()			pinAFIO(PA3, AFIO4)
#define U2TX2PA14()			pinAFIO(PA14,AFIO4)		//SWCLK pin
#define U2RX2PA15()			pinAFIO(PA15,AFIO4)		

//pwm / oc
//global defines
#define TIM1CH1toPA8()				pinAFIO(PA8, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 8))) | (/*0b10*/0x02<<(2* 8)); GPIOA->OTYPER &=~(1<< 8); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 8))) | (/*0b01*/0x01<<(2* 8)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM1CH1toPA3()				pinAFIO(PA3, AFIO13)//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 8))) | (/*0b10*/0x02<<(2* 8)); GPIOA->OTYPER &=~(1<< 8); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 8))) | (/*0b01*/0x01<<(2* 8)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM1CH2toPA9()				pinAFIO(PA9, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM1CH2toPA13()				pinAFIO(PA13,AFIO13)//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM1CH2toPB3()				pinAFIO(PB3, AFIO1) //do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM1CH3toPA0()				pinAFIO(PA0, AFIO13)//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (2<<(4*(10%8)));} while (0)
#define TIM1CH3toPA10()				pinAFIO(PA10,AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (2<<(4*(10%8)));} while (0)
#define TIM1CH3toPB6()				pinAFIO(PB6, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*10))) | (/*0b10*/0x02<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*10))) | (/*0b01*/0x01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (2<<(4*(10%8)));} while (0)
#define TIM1CH4toPA1()				pinAFIO(PA1, AFIO13)//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2*11))) | (/*0b10*/0x02<<(2*11)); GPIOA->OTYPER &=~(1<<11); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2*11))) | (/*0b01*/0x01<<(2*11)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(11%8)))) | (2<<(4*(11%8)));} while (0)
#define TIM1CH1NtoPA0()				pinAFIO(PA0, AFIO14)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)
#define TIM1CH1NtoPA7()				pinAFIO(PA7, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)
#define TIM1CH1NtoPB13()			pinAFIO(PB13,AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*13))) | (/*0b10*/0x02<<(2*13)); GPIOB->OTYPER &=~(1<<13); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*13))) | (/*0b01*/0x01<<(2*13)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(13%8)))) | (2<<(4*(13%8)));} while (0)
#define TIM1CH2NtoPA1()				pinAFIO(PA1, AFIO14)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)
#define TIM1CH2NtoPB0()				pinAFIO(PB0, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 0))) | (/*0b10*/0x02<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 0))) | (/*0b01*/0x01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (2<<(4*( 0%8)));} while (0)
#define TIM1CH3NtoPB1()				pinAFIO(PB1, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (2<<(4*( 1%8)));} while (0)

//MODER=/*0b10*/0x02 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 1, AFR[0]/low byte
#define TIM3CH1toPA2()				pinAFIO(PA2, AFIO13)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (1<<(4*( 6%8)));} while (0)
#define TIM3CH1toPA6()				pinAFIO(PA6, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (1<<(4*( 6%8)));} while (0)
#define TIM3CH1toPB4()				pinAFIO(PB4, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 4))) | (/*0b10*/0x02<<(2* 4)); GPIOB->OTYPER &=~(1<< 4); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 4))) | (/*0b01*/0x01<<(2* 4)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 4%8)))) | (1<<(4*( 4%8)));} while (0)
#define TIM3CH2toPA5()				pinAFIO(PA5, AFIO13)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (1<<(4*( 7%8)));} while (0)
#define TIM3CH2toPA7()				pinAFIO(PA7, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (1<<(4*( 7%8)));} while (0)
#define TIM3CH2toPB5()				pinAFIO(PB5, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 5))) | (/*0b10*/0x02<<(2* 5)); GPIOB->OTYPER &=~(1<< 5); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 5))) | (/*0b01*/0x01<<(2* 5)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 5%8)))) | (1<<(1*( 5%8)));} while (0)
#define TIM3CH3toPB0()				pinAFIO(PB0, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 0))) | (/*0b10*/0x02<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 0))) | (/*0b01*/0x01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (1<<(4*( 0%8)));} while (0)
#define TIM3CH3toPF3()				pinAFIO(PF3, AFIO13)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 0))) | (/*0b10*/0x02<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 0))) | (/*0b01*/0x01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (1<<(4*( 0%8)));} while (0)
#define TIM3CH4toPB1()				pinAFIO(PB1, AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (1<<(4*( 1%8)));} while (0)

#define TIM14CH1toPA4()				pinAFIO(PA4, AFIO4)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 4))) | (/*0b10*/0x02<<(2* 4)); GPIOA->OTYPER &=~(1<< 4); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 4))) | (/*0b01*/0x01<<(2* 4)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 4%8)))) | (4<<(4*( 4%8)));} while (0)
#define TIM14CH1toPA7()				pinAFIO(PA7, AFIO4)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (4<<(4*( 7%8)));} while (0)
#define TIM14CH1toPB1()				pinAFIO(PB1, AFIO0)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (0<<(4*( 1%8)));} while (0)
#define TIM14CH1toPF0()				pinAFIO(PF0, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (0<<(4*( 1%8)));} while (0)
#define TIM14CH1toPF1()				pinAFIO(PF1, AFIO13)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 1))) | (/*0b10*/0x02<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 1))) | (/*0b01*/0x01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (0<<(4*( 1%8)));} while (0)

//MODER=/*0b10*/0x02 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 0, AFR[0]/low byte
#define TIM15CH1toPA2()				pinAFIO(PA2, AFIO0)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 2))) | (/*0b10*/0x02<<(2* 2)); GPIOA->OTYPER &=~(1<< 2); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 2))) | (/*0b01*/0x01<<(2* 2)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 2%8)))) | (0<<(4*( 2%8)));} while (0)
#define TIM15CH2toPA3()				pinAFIO(PA3, AFIO0)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 3))) | (/*0b10*/0x02<<(2* 3)); GPIOA->OTYPER &=~(1<< 3); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 3))) | (/*0b01*/0x01<<(2* 3)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 3%8)))) | (0<<(4*( 3%8)));} while (0)
//AF=1, AFR[1]/high byte
#define TIM15CH1toPB14()			pinAFIO(PB14,AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*14))) | (/*0b10*/0x02<<(2*14)); GPIOB->OTYPER &=~(1<<14); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*14))) | (/*0b01*/0x01<<(2*14)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(14%8)))) | (1<<(4*(14%8)));} while (0)
#define TIM15CH2toPB15()			pinAFIO(PB15,AFIO1)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOB->OTYPER &=~(1<<15); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(15%8)))) | (1<<(4*(15%8)));} while (0)
#define TIM15CH1NtoPB15()			pinAFIO(PB15,AFIO3)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2*15))) | (/*0b10*/0x02<<(2*15)); GPIOB->OTYPER &=~(1<<15); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2*15))) | (/*0b01*/0x01<<(2*15)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(15%8)))) | (3<<(4*(15%8)));} while (0)

//MODER=/*0b10*/0x02 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 5, AFR[0]/low byte
#define TIM16CH1toPA6()				pinAFIO(PA6, AFIO5)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (5<<(4*( 6%8)));} while (0)
//AF2, AFR[1]/high byte
#define TIM16CH1toPB8()				pinAFIO(PB8, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 8))) | (/*0b10*/0x02<<(2* 8)); GPIOB->OTYPER &=~(1<< 8); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 8))) | (/*0b01*/0x01<<(2* 8)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM16CH1NtoPB6()			pinAFIO(PB6, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 6))) | (/*0b10*/0x02<<(2* 6)); GPIOB->OTYPER &=~(1<< 6); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 6))) | (/*0b01*/0x01<<(2* 6)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 6%8)))) | (2<<(4*( 6%8)));} while (0)

//MODER=/*0b10*/0x02 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 5, AFR[0]/low byte
#define TIM17CH1toPA7()				pinAFIO(PA7, AFIO5)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (5<<(4*( 7%8)));} while (0)
//AF2, AFR[1]/high byte
#define TIM17CH1toPB8()				pinAFIO(PB8, AFIO13)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 9))) | (/*0b10*/0x02<<(2* 9)); GPIOB->OTYPER &=~(1<< 9); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 9))) | (/*0b01*/0x01<<(2* 9)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM17CH1NtoPB7()			pinAFIO(PB7, AFIO2)	//do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(/*0b11*/0x03<<(2* 7))) | (/*0b10*/0x02<<(2* 7)); GPIOB->OTYPER &=~(1<< 7); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(/*0b11*/0x03<<(2* 7))) | (/*0b01*/0x01<<(2* 7)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 7%8)))) | (2<<(4*( 7%8)));} while (0)

//spi pins
#define NSS1toPA4()					pinAFIO(PA4, AFIO0)
#define NSS1toPA15()				pinAFIO(PA15,AFIO0)
#define NSS1toPB12()				pinAFIO(PB12,AFIO0)
#define SCK1toPA5()					pinAFIO(PA5, AFIO0)
#define SCK1toPB3()					pinAFIO(PB3, AFIO0)
#define SCK1toPB13()				pinAFIO(PB13,AFIO0)
#define MOSI1toPA7()				pinAFIO(PA7, AFIO0)
#define MOSI1toPB5()				pinAFIO(PB5, AFIO0)
#define MOSI1toPB15()				pinAFIO(PB15,AFIO0)
#define MISO1toPA6()				pinAFIO(PA6, AFIO0)
#define MISO1toPB4()				pinAFIO(PB4, AFIO0)
#define MISO1toPB14()				pinAFIO(PB14,AFIO0)

//i2c1 pins
#define SCL1toPA3()					pinAFIO(PA3, AFIO12)
#define SCL1toPA8()					pinAFIO(PA8, AFIO12)
#define SCL1toPA9()					pinAFIO(PA9, AFIO6)
#define SCL1toPA10()				pinAFIO(PA10,AFIO12)
#define SCL1toPA11()				pinAFIO(PA11,AFIO6)
#define SCL1toPB6()					pinAFIO(PB6, AFIO6)
#define SCL1toPB8()					pinAFIO(PB8, AFIO6)
#define SCL1toPF1()					pinAFIO(PF1, AFIO12)
#define SDA1toPA2()					pinAFIO(PA2, AFIO12)
#define SDA1toPA7()					pinAFIO(PA7, AFIO12)
#define SDA1toPA9()					pinAFIO(PA9, AFIO12)
#define SDA1toPA10()				pinAFIO(PA10,AFIO6)
#define SDA1toPA12()				pinAFIO(PA12,AFIO6)
#define SDA1toPB7()					pinAFIO(PB7, AFIO6)
#define SDA1toPB8()					pinAFIO(PB8, AFIO12)
#define SDA1toPF0()					pinAFIO(PF0, AFIO12)

//others
#define MCOtoPA1()					pinAFIO(PA1, AFIO15)	//mco on pa1@af15
#define MCOtoPA5()					pinAFIO(PA5, AFIO15)	//mco on pa5@af15
#define MCOtoPA8()					pinAFIO(PA8, AFIO5)		//mco on pa8@af5
#define MCOtoPA9()					pinAFIO(PA9, AFIO5)		//mco on pa9@af5
#define MCOtoPA13()					pinAFIO(PA13,AFIO15)	//mco on pa13@af15
#define MCOtoPA14()					pinAFIO(PA14,AFIO15)	//mco on pa14@af15
#define MCOtoPF2()					pinAFIO(PF2, AFIO6)		//mco on pf2@af6

//swc(pad14)/swd(pa13) pins
#define SWDtoPA13()					pinAFIO(PA13,AFIO0)
#define SWCtoPA14()					pinAFIO(PA14,AFIO0)
#define PA13toSWD()					SWDtoPA13()				//pinAFIO(PA13,AFIO0)
#define PA14toSWC()					SWCtoPA14()				//pinAFIO(PA14,AFIO0)

//end pin config

//for compatability with CoIDE
/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT   ((uint16_t)0x5000) /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */

/**
 * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup 
   Timeout value 
   */
#if !defined  (HSI_STARTUP_TIMEOUT)
#define HSI_STARTUP_TIMEOUT   ((uint16_t)0x5000) /*!< Time out for HSI start up */
#endif /* HSI_STARTUP_TIMEOUT */


//end compatability with CoIDE
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

//port/gpio oriented macros
//operate on a var/register
#define REG_EQU(var, bits)					var  = (bits)
#define REG_SET(var, bits)					var |= (bits)
#define REG_CLR(var, bits)					var &=~(bits)
#define REG_FLP(var, bits)					var ^= (bits)
#define REG_GET(var, bits)					((var) & (bits))
#define REG_MOD(var, bits, mask)			(var = ((var) &~(mask)) | ((bits) & (mask)))

//set a pin to output/input
#define IO_SET(port, pins)					REG_SET(port->ODR, pins)		//set bits
#define IO_CLR(port, pins)					REG_CLR(port->ODR, pins)		//clear bits
#define IO_FLP(port, pins)					REG_FLP(port->ODR, pins)		//flip bits
#define IO_GET(port, pins)					REG_GET(port->IDR, pins)		//read bits
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
#define GIO_SET(port, pins)					IO_SET(port, pins)				//REG_SET(port->ODR, pins)
#define GIO_CLR(port, pins)					IO_CLR(port, pins)				//REG_CLR(port->ODR, pins)
#define GIO_FLP(port, pins)					IO_FLP(port, pins)				//REG_FLP(port->ODR, pins)
#define GIO_GET(port, pins)					IO_GET(port, pins)
#define GIO_IN(port, pins)					IO_IN(port, pins)
#define GIO_INFL(port, pins)				IO_INFL(port, pins)
#define GIO_INPU(port, pins)				IO_INPU(port, pins)
#define GIO_INPD(port, pins)				IO_INPD(port, pins)
#define GIO_OUT(port, pins)					IO_OUT(port, pins)
#define GIO_OUTOD(port, pins)				IO_OUTOD(port, pins)
#define GIO_OUTPP(port, pins)				IO_OUTPP(port, pins)
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

//port structure

//global defines

//pin enum - matches GPIO_PinDef[]
typedef enum {
#if defined(GPIOA)
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
#endif		//GPIOA
#if defined(GPIOB)
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#endif 		//GPIOB
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
#define OUTPUT_OD			3			//(!INPUT)
#define INPUT_PULLDW		4
#define INPUT_PU			INPUT_PULLUP
#define INPUT_PD			INPUT_PULLDW

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
	AFIO8,
	AFIO9,
	AFIO10,
	AFIO11,
	AFIO12,
	AFIO13,
	AFIO14,
	AFIO15,
	AFIO1MAX,
} AFIO_TypeDef;

void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
int digitalRead(PIN_TypeDef pin);
#define pinFlip(pin)		digitalWrite(pin, !digitalRead(pin))
void pinAFIO(PIN_TypeDef pin, AFIO_TypeDef AFIOn);	//configure pin for AFIO

//do not put SWD pins (PA13/14) into GPIO - you will lose debug capability
//if you do, recover as follows:
// 1. pull down the nRST pin and hold it down.
// 2. flash the chip
// 3. release the nRST pin
//
// step 2 and 3 should be done reasonably quickly (within seconds of each other)
//

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

//global defines
#define IDCODE				(DBGMCU->IDCODE)
#define CPUID				(SCB->CPUID)
//128byte unique id
//usage: UID[0]..UID[127]
#define UID					((char *) 0x1fff0e00)


//global variables

//empty interrupt handler
void empty_handler(void);


//reset the mcu
void mcuInit(void);

//activate the isr handler
void systickAttachISR(void (*isrptr)(void));

//#define Mhz					000000ul	//suffix for Mhz
#define F_UART				(F_PHB)	//8Mhz		//crystal frequency
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
#define uart1bps()			(F_UART / USART1->BRR)
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
#define uart2bps()			(F_UART / USART2->BRR)
#define u2bps()				uart2bps()
//end Serial

//timers
#define tmrCC1Get(tmr)		(tmr->CCR1)
#define tmrCC2Get(tmr)		(tmr->CCR2)
#define tmrCC3Get(tmr)		(tmr->CCR3)
#define tmrCC4Get(tmr)		(tmr->CCR4)
//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps);							//initialize timer with a prescaler
#define tmr1Get()			(TIM1->CNT)
#define tmr1SetPS(ps)		TIM1->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr1GetPS()			(TIM1->PSC + 1l)		
#define tmr1SetPR(pr)		TIM1->ARR = (pr) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr1GetPR()			(TIM1->ARR + 1l)		
void tmr1OVFAttachISR(void (*isrptr)(void));		//activate the ovf isr handler
//output compare
void tmr1OC1SetPR(uint16_t pr);						//set period
void tmr1OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC2SetPR(uint16_t pr);						//set period
void tmr1OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC3SetPR(uint16_t pr);						//set period
void tmr1OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr1OC4SetPR(uint16_t pr);						//set period
void tmr1OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr1IC1Init(uint8_t ps);								//activate input capture
#define tmr1CC1Get()		tmrCC1Get(TIM1)
void tmr1IC2Init(uint8_t ps);								//activate input capture
#define tmr1CC2Get()		tmrCC2Get(TIM1)
void tmr1IC3Init(uint8_t ps);								//activate input capture
#define tmr1CC3Get()		tmrCC3Get(TIM1)
void tmr1IC4Init(uint8_t ps);								//activate input capture
#define tmr1CC4Get()		tmrCC4Get(TIM1)

//TIM2 does not exist on PY32
//#define TIM2_BASE             (APBPERIPH_BASE + 0x0000ul)
//#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
//#define TIM2_IRQn			15
#define RCC_APBENR1_TIM2EN_Pos           (0U)
#define RCC_APBENR1_TIM2EN_Msk           (0x1UL << RCC_APBENR1_TIM2EN_Pos)      /*!< 0x00000001 */
#define RCC_APBENR1_TIM2EN               RCC_APBENR1_TIM2EN_Msk
void tmr2Init(uint16_t ps);							//initialize timer with a prescaler
#define tmr2Get()			(TIM2->CNT)
#define tmr2SetPS(ps)		TIM2->PSC = (ps) - 1	//void tmr2SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr2GetPS()			(TIM2->PSC + 1l)		
#define tmr2SetPR(pr)		TIM2->ARR = (pr) - 1	//void tmr2SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr2GetPR()			(TIM2->ARR + 1l)		
void tmr2OVFAttachISR(void (*isrptr)(void));		//activate the ovf isr handler
//output compare
void tmr2OC1SetPR(uint16_t pr);						//set period
void tmr2OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC2SetPR(uint16_t pr);						//set period
void tmr2OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC3SetPR(uint16_t pr);						//set period
void tmr2OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr2OC4SetPR(uint16_t pr);						//set period
void tmr2OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr2IC1Init(uint8_t ps);								//activate input capture
#define tmr2CC1Get()		tmrCC1Get(TIM2)
void tmr2IC2Init(uint8_t ps);								//activate input capture
#define tmr2CC2Get()		tmrCC2Get(TIM2)
void tmr2IC3Init(uint8_t ps);								//activate input capture
#define tmr2CC3Get()		tmrCC3Get(TIM2)
void tmr2IC4Init(uint8_t ps);								//activate input capture
#define tmr2CC4Get()		tmrCC4Get(TIM2)

//initialize the timer3 (16bit)
void tmr3Init(uint16_t ps);							//initialize timer with a prescaler
#define tmr3Get()			(TIM3->CNT)
#define tmr3SetPS(ps)		TIM3->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr3GetPS()			(TIM3->PSC + 1l)		
#define tmr3SetPR(pr)		TIM3->ARR = (pr) - 1	//void tmr3SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr3GetPR()			(TIM3->ARR + 1l)		
void tmr3OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC1SetPR(uint16_t pr);						//set period
void tmr3OC1AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC2SetPR(uint16_t pr);						//set period
void tmr3OC2AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC3SetPR(uint16_t pr);						//set period
void tmr3OC3AttachISR(void (*isrptr)(void));		//activate the isr handler
void tmr3OC4SetPR(uint16_t pr);						//set period
void tmr3OC4AttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr3IC1Init(uint8_t ps);								//activate input capture
#define tmr3CC1Get()		tmrCC1Get(TIM3)
void tmr3IC2Init(uint8_t ps);								//activate input capture
#define tmr3CC2Get()		tmrCC2Get(TIM3)
void tmr3IC3Init(uint8_t ps);								//activate input capture
#define tmr3CC3Get()		tmrCC3Get(TIM3)
void tmr3IC4Init(uint8_t ps);								//activate input capture
#define tmr3CC4Get()		tmrCC4Get(TIM3)

//initialize the timer6 (16bit)
void tmr6Init(uint16_t ps);							//initialize timer with a prescaler
#define tmr6Get()			(TIM6->CNT)
#define tmr6SetPS(ps)		TIM6->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr6GetPS()			(TIM6->PSC + 1l)		
#define tmr6SetPR(pr)		TIM6->ARR = (pr) - 1	//void tmr6SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr6GetPR()			(TIM6->ARR + 1l)		
void tmr6OVFAttachISR(void (*isrptr)(void));		//activate the isr handler
//input capture
void tmr6IC1Init(uint8_t ps);								//activate input capture
#define tmr6CC1Get()		tmrCC1Get(TIM6)
void tmr6IC2Init(uint8_t ps);								//activate input capture
#define tmr6CC2Get()		tmrCC2Get(TIM6)
void tmr6IC3Init(uint8_t ps);								//activate input capture
#define tmr6CC3Get()		tmrCC3Get(TIM6)
void tmr6IC4Init(uint8_t ps);								//activate input capture
#define tmr6CC4Get()		tmrCC4Get(TIM6)

//initialize the timer14 (16bit)
void tmr14Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr14Get()			(TIM14->CNT)
#define tmr14SetPS(ps)		TIM14->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr14GetPS()		(TIM14->PSC + 1l)		
#define tmr14SetPR(pr)		TIM14->ARR = (pr) - 1	//void tmr14SetPR(uint16_t pr);						//set period
#define tmr14GetPR()		(TIM14->ARR + 1l)		
void tmr14OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output compare
void tmr14OC1SetPR(uint16_t pr);					//set period
void tmr14OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr14OC2SetPR(uint16_t pr);					//set period
void tmr14OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr14IC1Init(uint8_t ps);								//activate input capture
#define tmr14CC1Get()		tmrCC1Get(TIM14)
void tmr14IC2Init(uint8_t ps);								//activate input capture
#define tmr14CC2Get()		tmrCC2Get(TIM14)
void tmr14IC3Init(uint8_t ps);								//activate input capture
#define tmr14CC3Get()		tmrCC3Get(TIM14)
void tmr14IC4Init(uint8_t ps);								//activate input capture
#define tmr14CC4Get()		tmrCC4Get(TIM14)

//initialize the timer15 (16bit)
void tmr15Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr15Get()			(TIM15->CNT)
#define tmr15SetPS(ps)		TIM15->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr15GetPS()		(TIM15->PSC + 1l)		
#define tmr15SetPR(pr)		TIM15->ARR = (pr) - 1	//void tmr15SetPR(uint16_t pr);						//set period
#define tmr15GetPR()		(TIM15->ARR + 1l)		
void tmr15OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output compare
void tmr15OC1SetPR(uint16_t pr);					//set period
void tmr15OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr15OC2SetPR(uint16_t pr);					//set period
void tmr15OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr15IC1Init(uint8_t ps);								//activate input capture
#define tmr15CC1Get()		tmrCC1Get(TIM15)
void tmr15IC2Init(uint8_t ps);								//activate input capture
#define tmr15CC2Get()		tmrCC2Get(TIM15)
void tmr15IC3Init(uint8_t ps);								//activate input capture
#define tmr15CC3Get()		tmrCC3Get(TIM15)
void tmr15IC4Init(uint8_t ps);								//activate input capture
#define tmr15CC4Get()		tmrCC4Get(TIM15)

//initialize the timer16 (16bit)
void tmr16Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr16Get()			(TIM16->CNT)
#define tmr16SetPS(ps)		TIM16->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr16GetPS()		(TIM16->PSC + 1l)		
#define tmr16SetPR(pr)		TIM16->ARR = (pr) - 1	//void tmr16SetPR(uint16_t pr);						//set period
#define tmr16GetPR()		(TIM16->ARR + 1l)		
void tmr16OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output compare
void tmr16OC1SetPR(uint16_t pr);					//set period
void tmr16OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
void tmr16OC2SetPR(uint16_t pr);					//set period
void tmr16OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr16IC1Init(uint8_t ps);								//activate input capture
#define tmr16CC1Get()		tmrCC1Get(TIM16)
void tmr16IC2Init(uint8_t ps);								//activate input capture
#define tmr16CC2Get()		tmrCC2Get(TIM16)
void tmr16IC3Init(uint8_t ps);								//activate input capture
#define tmr16CC3Get()		tmrCC3Get(TIM16)
void tmr16IC4Init(uint8_t ps);								//activate input capture
#define tmr16CC4Get()		tmrCC4Get(TIM16)

//initialize the timer17 (16bit)
void tmr17Init(uint16_t ps);						//initialize timer with a prescaler
#define tmr17Get()			(TIM17->CNT)
#define tmr17SetPS(ps)		TIM17->PSC = (ps) - 1	//void tmr1SetPR(uint16_t pr);						//set period for ovf interupt
#define tmr17GetPS()		(TIM17->PSC + 1l)		
#define tmr17SetPR(pr)		TIM17->ARR = (pr) - 1	//void tmr17SetPR(uint16_t pr);						//set period
#define tmr17GetPR()		(TIM17->ARR + 1l)		
void tmr17OVFAttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//output compare
void tmr17OC1SetPR(uint16_t pr);					//set period
void tmr17OC1AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//void tmr17OC2SetPR(uint16_t pr);					//set period
//void tmr17OC2AttachISR(void (*isrptr)(void));		//activate the isr overflow handler
//input capture
void tmr17IC1Init(uint8_t ps);								//activate input capture
#define tmr17CC1Get()		tmrCC1Get(TIM17)
void tmr17IC2Init(uint8_t ps);								//activate input capture
#define tmr17CC2Get()		tmrCC2Get(TIM17)
void tmr17IC3Init(uint8_t ps);								//activate input capture
#define tmr17CC3Get()		tmrCC3Get(TIM17)
void tmr17IC4Init(uint8_t ps);								//activate input capture
#define tmr17CC4Get()		tmrCC4Get(TIM17)

//initialize tim31 as a synchronized 32-bit timer
//tim3 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr31Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr31GetPS()		tmr3GetPS()
#define tmr31SetPS(ps)		tmr3SetPS(ps)
uint32_t tmr31Get(void);				//read 32-bit timer value
uint32_t tmr31SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr31GetPR(void);				//read 32-bit timer period
#define tmr31OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

void tmr171Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr171GetPS()		tmr17GetPS()
#define tmr171SetPS(ps)		tmr17SetPS(ps)
uint32_t tmr171Get(void);				//read 32-bit timer value
uint32_t tmr171SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr171GetPR(void);			//read 32-bit timer period
#define tmr171OVFAttachISR(isrptr)		tmr1OVFAttachISR(isrptr)		//activate the isr handler

void tmr13Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr13GetPS()		tmr1GetPS()
#define tmr13SetPS(ps)		tmr1SetPS(ps)
uint32_t tmr13Get(void);				//read 32-bit timer value
uint32_t tmr13SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr13GetPR(void);				//read 32-bit timer period
#define tmr13OVFAttachISR(isrptr)		tmr3OVFAttachISR(isrptr)		//activate the isr handler

void tmr143Init(uint16_t ps);			//initialize tmr, default period is 0x100000000ul
#define tmr143GetPS()		tmr14GetPS()
#define tmr143SetPS(ps)		tmr14SetPS(ps)
uint32_t tmr143Get(void);				//read 32-bit timer value
uint32_t tmr143SetPR(uint32_t pr);		//set 32-bit timer period
uint32_t tmr143GetPR(void);				//read 32-bit timer period
#define tmr143OVFAttachISR(isrptr)		tmr3OVFAttachISR(isrptr)		//activate the isr handler

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm1Init(uint16_t TxCCP_PS);
#define pwm1GetPS()			tmr1GetPS()
#define pwm1SetPS(ps)		tmr1SetPS(ps)
#define pwm1GetPR()			tmr1GetPR()
#define pwm1SetPR(pr)		tmr1SetPR(pr)
//set duty cycle
#define pwm1SetDC1(dc)		TIM1->CCR1 = (dc)
#define pwm1SetDC2(dc)		TIM1->CCR2 = (dc)
#define pwm1SetDC3(dc)		TIM1->CCR3 = (dc)
#define pwm1SetDC4(dc)		TIM1->CCR4 = (dc)
#define pwm1GetDC1()		(TIM1->CCR1)
#define pwm1GetDC2()		(TIM1->CCR2)
#define pwm1GetDC3()		(TIM1->CCR3)
#define pwm1GetDC4()		(TIM1->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm2Init(uint16_t TxCCP_PS);
#define pwm2GetPS()			tmr2GetPS()
#define pwm2SetPS(ps)		tmr2SetPS(ps)
#define pwm2GetPR()			tmr2GetPR()
#define pwm2SetPR(pr)		tmr2SetPR(pr)
//set duty cycle
#define pwm2SetDC1(dc)		TIM2->CCR1 = (dc)
#define pwm2SetDC2(dc)		TIM2->CCR2 = (dc)
#define pwm2SetDC3(dc)		TIM2->CCR3 = (dc)
#define pwm2SetDC4(dc)		TIM2->CCR4 = (dc)
#define pwm2GetDC1()		(TIM2->CCR1)
#define pwm2GetDC2()		(TIM2->CCR2)
#define pwm2GetDC3()		(TIM2->CCR3)
#define pwm2GetDC4()		(TIM2->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm3Init(uint16_t TxCCP_PS);
#define pwm3GetPS()			tmr3GetPS()
#define pwm3SetPS(ps)		tmr3SetPS(ps)
#define pwm3GetPR()			tmr3GetPR()
#define pwm3SetPR(pr)		tmr3SetPR(pr)
//set duty cycle
#define pwm3SetDC1(dc)		TIM3->CCR1 = (dc)
#define pwm3SetDC2(dc)		TIM3->CCR2 = (dc)
#define pwm3SetDC3(dc)		TIM3->CCR3 = (dc)
#define pwm3SetDC4(dc)		TIM3->CCR4 = (dc)
#define pwm3GetDC1()		(TIM3->CCR1)
#define pwm3GetDC2()		(TIM3->CCR2)
#define pwm3GetDC3()		(TIM3->CCR3)
#define pwm3GetDC4()		(TIM3->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm14Init(uint16_t TxCCP_PS);
#define pwm14GetPS()		tmr14GetPS()
#define pwm14SetPS(ps)		tmr14SetPS(ps)
#define pwm14GetPR()		tmr14GetPR()
#define pwm14SetPR(pr)		tmr14SetPR(pr)
//set duty cycle
#define pwm14SetDC1(dc)		TIM14->CCR1 = (dc)
#define pwm14SetDC2(dc)		TIM14->CCR2 = (dc)
#define pwm14SetDC3(dc)		TIM14->CCR3 = (dc)
#define pwm14SetDC4(dc)		TIM14->CCR4 = (dc)
#define pwm14GetDC1()		(TIM14->CCR1)
#define pwm14GetDC2()		(TIM14->CCR2)
#define pwm14GetDC3()		(TIM14->CCR3)
#define pwm14GetDC4()		(TIM14->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS);
#define pwm15GetPS()		tmr15GetPS()
#define pwm15SetPS(ps)		tmr15SetPS(ps)
#define pwm15GetPR()		tmr15GetPR()
#define pwm15SetPR(pr)		tmr15SetPR(pr)
//set duty cycle
#define pwm15SetDC1(dc)		TIM15->CCR1 = (dc)
#define pwm15SetDC2(dc)		TIM15->CCR2 = (dc)
#define pwm15SetDC3(dc)		TIM15->CCR3 = (dc)
#define pwm15SetDC4(dc)		TIM15->CCR4 = (dc)
#define pwm15GetDC1()		(TIM15->CCR1)
#define pwm15GetDC2()		(TIM15->CCR2)
#define pwm15GetDC3()		(TIM15->CCR3)
#define pwm15GetDC4()		(TIM15->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm16Init(uint16_t TxCCP_PS);
#define pwm16GetPS()		tmr16GetPS()
#define pwm16SetPS(ps)		tmr16SetPS(ps)
#define pwm16GetPR()		tmr16GetPR()
#define pwm16SetPR(pr)		tmr16SetPR(pr)
//set duty cycle
#define pwm16SetDC1(dc)		TIM16->CCR1 = (dc)
#define pwm16SetDC2(dc)		TIM16->CCR2 = (dc)
#define pwm16SetDC3(dc)		TIM16->CCR3 = (dc)
#define pwm16SetDC4(dc)		TIM16->CCR4 = (dc)
#define pwm16GetDC1()		(TIM16->CCR1)
#define pwm16GetDC2()		(TIM16->CCR2)
#define pwm16GetDC3()		(TIM16->CCR3)
#define pwm16GetDC4()		(TIM16->CCR4)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm17Init(uint16_t TxCCP_PS);
#define pwm17GetPS()		tmr17GetPS()
#define pwm17SetPS(ps)		tmr17SetPS(ps)
#define pwm17GetPR()		tmr17GetPR()
#define pwm17SetPR(pr)		tmr17SetPR(pr)
//set duty cycle
#define pwm17SetDC1(dc)		TIM17->CCR1 = (dc)
#define pwm17SetDC2(dc)		TIM17->CCR2 = (dc)
#define pwm17SetDC3(dc)		TIM17->CCR3 = (dc)
#define pwm17SetDC4(dc)		TIM17->CCR4 = (dc)
#define pwm17GetDC1()		(TIM17->CCR1)
#define pwm17GetDC2()		(TIM17->CCR2)
#define pwm17GetDC3()		(TIM17->CCR3)
#define pwm17GetDC4()		(TIM17->CCR4)

//lptimer

//initialize lptimer
void lptmr1Init(uint8_t ps);					//initialize lptimer1. ps=0..7->1x..128x
void lptmr1AttachISR(void (*isr_ptr)(void));	//install user handler
#define lptmr1Start()	LPTIM1->CR |= LPTIM_CR_SNGSTRT
#define lptmr1OVF()		(LPTIM1->ISR & LPTIM_ISR_ARRM)
#define lptmr1SetPR(pr)	LPTIM1->ARR = (pr) - 1
#define lptmr1GetPR()	(LPTIM1->ARR + 1ul)
#define lptmr1SetPS(ps)	LPTIM1->CFGR = (LPTIM1->CFGR &~LPTIM_CFGR_PRESC) | (((ps) << LPTIM_CFGR_PRESC_Pos) & LPTIM_CFGR_PRESC)
#define lptmr1GetPS()	((LPTIM1->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos)
//end lptimer

//adc
//adc channel definitions
#define ADC_CH0			0
#define ADC_CH1			1
#define ADC_CH2			2
#define ADC_CH3			3
#define ADC_CH4			4
#define ADC_CH5			5
#define ADC_CH6			6
#define ADC_CH7			7
#define ADC_CH8			8
#define ADC_CH9			9
//#define ADC_CH10		10
#define ADC_CH11		11
#define ADC_CH12		12
//#define ADC_CH13		(1ul<<13)
//#define ADC_CH14		(1ul<<14)
//#define ADC_CH15		(1ul<<15)
//#define ADC_CH16		(1ul<<16)
//#define ADC_CH17		(1ul<<17)
#define ADC_TS			ADC_CH11				//tempeature sensor on CH11
#define ADC_VREFINT		ADC_CH12				//internal reference on CH12 @ 1.20v
#define ADC_VREF		ADC_VREFINT				//internal reference on CH12 @ 1.20v
#define ADC_TEMP		ADC_TS					//internal reference on CH11 @ 1.20v
#define ADC_ARAND		ADC_VREFINT				//the channel for random number generators

//calibration for temperature sensors, 12 bits
#define TSCAL1			((*(int32_t *) 0x1fff0f14) & 0x0fff)		//~941 @ VddA=3.3v
#define TSCAL2			((*(int32_t *) 0x1fff0f18) & 0x0fff)		//~1130

//convert temperature sensor adc reading into temperaturex10
//two alternatives to calculate temperature. both work
#define ADC2Cx100(adc)		((8500 - 3000) * ((adc) * VDDAmv / 3000 - TSCAL1) / (TSCAL2 - TSCAL1) + 3000)	//per reference manual - need to correct adc reading due to VDDA differences
//#define ADC2Cx100(adc)		((760 - ADC2mv(adc)) * 10 * 100 / (25) + 3000)					//per datasheet, using average slope. 
#define ADC2Fx100(adc)		(ADC2Cx100(adc) * 9 / 5 + 3200)
#define VREFadc				1487l					//VREF ADC reading
#define VREFmv				1200l					//Vref voltage in mv
#define VDDAmv				(VREFmv * 4096 / VREFadc)	//VADD=3.3v
#define ADC2mv(adc)			((adc) * VDDAmv / 4096)	//convert 12-bit adc to mv
//rest the adc
//automatic sampling (ASAM=1), manual conversion
void adc1Init(void);
int16_t adc1Read(uint32_t adc_ch);

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
#define I2C_CMD_WRITE		0						//write command
#define I2C_CMD_READ		1						//read command

void i2c1Init(uint32_t br);							//i2c1 initialization
#define i2c1Busy()			(I2C1->SR2 & I2C_SR2_BUSY)
#define i2c1Wait()			while (i2c1Busy()) continue
#define i2c1Avail()			(I2C1->SR1 & I2C_SR1_RXNE)
#define i2c1Start()			do {I2C1->CR1 |= I2C_CR1_START; while (!I2C1->SR1 & I2C_SR1_SB) continue; I2C1->SR1;} while (0)
#define i2c1WriteAddr(dat)	do {I2C1->DR = (dat); while (!I2C1->SR1 & I2C_SR1_ADDR) continue; I2C1->SR1; I2C1->SR2;} while (0)
#define i2c1Write(dat)		do {I2C1->DR = (dat); while (!I2C1->SR1 & I2C_SR1_TXE) continue; } while (0)
#define i2c1Stop()			do {I2C1->CR1 |= I2C_CR1_STOP;} while (0)
#define i2c1Ack()			do {I2C1->CR1 |= I2C_CR1_ACK;} while (0)
#define i2c1NoAck()			do {I2C1->CR1 &=~I2C_CR1_ACK;} while (0)
int16_t i2c1Writes(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);	//WRITE_REG multiple bytes
int16_t i2c1Reads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);	//READ_BIT multiple bytes
//end i2c

//rtc
#define RCC_BDCR_RTCSEL_NOCLK		(0ul<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSE			(1ul<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSI			(2ul<<RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_HSE			(3ul<<RCC_BDCR_RTCSEL_Pos)
//#define RCC_BDCR_LSEON				(1ul<<0)
//#define RCC_BDCR_LSERDY				(1ul<<1)
//bcd to dec conversion
#define BCD2DEC(bcd)				((((bcd) & 0xf0)>>4)*10+((bcd) & 0x0f))		//convert bcd to int
#define DEC2BCD(dec)				(((((dec) / 10)<<4) +((dec) % 10)) & 0xff)	//convert int to bcd
//rtc
//reset the rtc
//ps = prescaler
//pr = period for alarm
//alarm isr is enabled
//rtcc_clk is clock source:
//RCC_BDCR_RTCSEL_LSI, RCC_BDCR_RTCSEL_LSE and RCC_BDCR_RTCSEL_HSE (HSE/128)
void RTCInit(uint32_t rtc_clk, uint32_t ps);	//initialize the RTCC
void RTCOVFAttachISR(void (*isr_ptr)(void));			//install rtc overflow isr
void RTCALRAttachISR(void (*isr_ptr)(void));			//install rtc alarm isr
void RTCSECAttachISR(void (*isr_ptr)(void));			//install rtc second isr
uint32_t RTCSetCNT(uint32_t val);				//set rtc counter
uint32_t RTCGetCNT(void);						//read rtc counter
uint32_t RTCSetALR(uint32_t val);				//set rtc alarm counter
uint32_t RTCGetALR(void);						//read rtc alarm counter
#define RTCGetSec()				RTCGetCNT()
#define RTCInitLSI()			RTCInit(RCC_BDCR_RTCSEL_LSI, F_LSI)
#define RTCInitLSE()			RTCInit(RCC_BDCR_RTCSEL_LSE, F_LSE)
#define RTCInitHSE()			RTCInit(RCC_BDCR_RTCSEL_HSE, F_HSE/128)
#define RTCTrimSet(trim)		do {PWR->CR1 |= PWR_CR1_DBP; RTC->BKP_RTCCR = (RTC->BKP_RTCCR &~BKP_RTCCR_CAL) | ((trim) & BKP_RTCCR_CAL); PWR->CR1 &=~PWR_CR1_DBP;	} while (0)
#define RTCTrimGet()			(RTC->BKP_RTCCR & BKP_RTCCR_CAL)
	
time_t RTC2time(time_t *t);						//read rtc into time_t
time_t time2RTC(time_t t);						//initialize rtc with time_t

uint32_t RTCGetPRL(void);						//read prll and prlh
uint32_t RTCGetDIV(void);						//read divl and divh
uint8_t RTCHalfsec(void);						//returns 0: if 1st half second, 1: if 2nd half second
//set calibration register
void RTCSetCal(uint8_t cal);					//set calibration: number of ticks to be ignored every 2^20 ticks (it slows down the rtc)
//end rtc

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
void extiInit(PIN_TypeDef PINx, uint8_t edge);
void extiAttachISR(PIN_TypeDef PINx, void (*isr_ptr)(void));		//install user handler
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
    uint16_t writeIndex;				//index for the current write-> advance first and then write
    uint16_t readIndex;					//index for the current read -> advance first and then read
} CB_TypeDef;

void cbInit(CB_TypeDef *cbuffer);		//reset the cbuffer
char cbEmpty(CB_TypeDef *cbuffer);		//returns true if cbuffer is empty
char cbFull(CB_TypeDef *cbuffer);		//returns true if cbuffer is full;
cbuffer_t cbWrite(CB_TypeDef *cbuffer, cbuffer_t dat);	//write to the buffer - needs to test if the buffer is full first
cbuffer_t cbRead(CB_TypeDef *cbuffer);	//read from the buffer - needs to test if the buffer is empty
void str2cb(CB_TypeDef *cb, char *str);	//write string to CB
void cb2str(char *str, CB_TypeDef *cb);	//write CB to string
//end circular buffer
#endif
