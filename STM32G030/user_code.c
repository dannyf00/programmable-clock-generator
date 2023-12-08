//STM32G030 - programmable clock generator / frequency divider
// - up to 6 independent channels of output frequencies (1x MCO + 5x TIMERs)
// - up to 6 Output pins: pin assignment and duty cycles are user programmable
// -   MCO:   PA9
// -   TIM1:  PA10
// -   TIM3:  PB0
// -   TIM14: PA4
// -   TIM16: PA6
// -   TIM17: PA7
// -   duty cycle: 50% on MCO output pin, user programmable on TIMER output pins
// - Frequency dividers: fully user programmable, via CLKGENn_DIV, n=0, 1, 3, 14, 16, 17
// - Clock source: 
// -   1. internal or external crystal (for programmable clock sources) or 
// -   2. external clocks (for frequency dividers)
// - Compiler: Keil MDK 5, PY32 DFP 1.2
//
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
//              |                 PA3 |>----> VDD
//              |                     |
//              |                 PA2 |>----> U2TX/LED2
//              |                     |
//              |                     |
//              |                     |//clkgen output pins: see py32duino.h for pin assignment
//              |                     |//all possible output pins listed in bracket
//              |                     |//not all output pins are available on your particular package
//              |                     |
//              |             PA9/16  |>----> MCO (PA8, PA9)
//              |                     |
//              |                     |>----> TIM1OC1 (50% dc) (PA8)
//              |                     |>----> TIM1OC2 (25% dc) (PB3, PA9)
//              |             PA10/17 |>----> TIM1OC3 (50% dc) (PB6, PA10)
//              |                     |>----> TIM1OC4 (75% dc) (PA11)
//              |                     |
//              |                     |>----> TIM3OC1 (50% dc) (PA6, PB4)
//              |                     |>----> TIM3OC2 (25% dc) (PA7, PB5)
//              |             PB0/15  |>----> TIM3OC3 (50% dc) (PB0)
//              |                     |>----> TIM3OC4 (75% dc) (PB1)
//              |                     |
//              |             PA4/11  |>----> TIM14OC1 (50% dc) (PB1, PA4, PA7)
//              |             PA6/13  |>----> TIM16OC1 (50% dc) (PB8, PA6)
//              |             PA7/14  |>----> TIM17OC1 (50% dc) (PB9, PA7)
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

#include "stm32duino.h"						//we use stm32g0

//hardware configuration
#define LED			PA6						//led pin
#define LED_DLY		(F_PHB / 2)				//half a second

#define VDD			PA3
#define GND			PA7						//pin grounded
//end hardware configuration

//global defines
#define CLKGEN0_DIV		7					//divider for clkgen0 / mco on HSI: 2's complements, from 0(1:1)...7(128:1)
#define CLKGEN1_DIV		10000				//divider for clkgen1 / tim1
//#define CLKGEN2_DIV		10000			//divider for clkgen2 / tim2 - not available on PY32F002A/PY32F030
#define CLKGEN3_DIV		20000				//divider for clkgen3 / tim3
#define CLKGEN14_DIV	20000				//divider for clkgen14 / tim14
#define CLKGEN16_DIV	20000				//divider for clkgen16 / tim16
#define CLKGEN17_DIV	20000				//divider for clkgen17 / tim17

//global variables

//clkgen0 - generator clock using mco
//initialize clkgen0 - output pin defined by MCOtoPIN() macro
#define clkgen0Get() (F_CPU >> CLKGEN0_DIV)	//return frequency
//div is 2's complements: 
uint32_t clkgen0Set(uint32_t div) {
	//configure MCO on systemclock
#if defined(MCOtoPIN)
	MCO2SYSCLK(div);						//MCO2HSE on systemclock
	MCOtoPIN();								//ENABLE output pin
#endif	//mco2pin
	return clkgen0Get();
}

//available timers are: TIM1, TIM3, TIM14, TIM16/TIM17
//clkgen1 - generate clock using tim1
//initialize clkgen1 - output pin defined by TIM1CHntoPIN() macros, n=1..4
#define clkgen1Get() (F_PHB / pwm1GetPS() / pwm1GetPR())	//return frequency
uint32_t clkgen1Set(uint32_t div) {
	pwm1Init(1); pwm1SetPR(div / pwm1GetPS());			//initialize clock, set prescaler
	pwm1SetDC1(pwm1GetPR() / 4 * 2);		//OC1 output at 50% duty cycle
	pwm1SetDC2(pwm1GetPR() / 4 * 1);		//OC2 output at 25% duty cycle
	pwm1SetDC3(pwm1GetPR() / 4 * 2);		//OC3 output at 50% duty cycle
	pwm1SetDC4(pwm1GetPR() / 4 * 3);		//OC4 output at 75% duty cycle
	return clkgen1Get();
}

//initialize clkgen3 - output pin defined by TIM3CHntoPIN() macros, n=1..4
#define clkgen3Get() (F_PHB / pwm3GetPS() / pwm3GetPR())	//return frequency
uint32_t clkgen3Set(uint32_t div) {
	pwm3Init(1); pwm3SetPR(div / pwm3GetPS());			//initialize clock, set prescaler
	pwm3SetDC1(pwm3GetPR() / 4 * 2);		//OC1 output at 50% duty cycle
	pwm3SetDC2(pwm3GetPR() / 4 * 1);		//OC2 output at 25% duty cycle
	pwm3SetDC3(pwm3GetPR() / 4 * 2);		//OC3 output at 50% duty cycle
	pwm3SetDC4(pwm3GetPR() / 4 * 3);		//OC4 output at 75% duty cycle
	return clkgen3Get();
}

//initialize clkgen14 - output pin defined by TIM14CH1toPIN() macro
#define clkgen14Get() (F_PHB / pwm14GetPS() / pwm14GetPR())	//return frequency
uint32_t clkgen14Set(uint32_t div) {
	pwm14Init(1); pwm14SetPR(div / pwm14GetPS());			//initialize clock, set prescaler
	pwm14SetDC1(pwm14GetPR() / 4 * 2);	//OC1 output at 50% duty cycle
	//pwm14SetDC2(pwm14GetPR() / 4 * 1);	//OC2 output at 25% duty cycle
	//pwm14SetDC3(pwm14GetPR() / 4 * 2);	//OC3 output at 50% duty cycle
	//pwm14SetDC4(pwm14GetPR() / 4 * 3);	//OC4 output at 75% duty cycle
	return clkgen14Get();
}

//initialize clkgen16 - output pin defined by TIM16CH1toPIN() macro
#define clkgen16Get() (F_PHB / pwm16GetPS() / pwm16GetPR())	//return frequency
uint32_t clkgen16Set(uint32_t div) {
	pwm16Init(1); pwm16SetPR(div / pwm16GetPS());			//initialize clock, set prescaler
	pwm16SetDC1(pwm16GetPR() / 4 * 2);	//OC1 output at 50% duty cycle
	//pwm16SetDC2(pwm16GetPR() / 4 * 1);	//OC2 output at 25% duty cycle
	//pwm16SetDC3(pwm16GetPR() / 4 * 2);	//OC3 output at 50% duty cycle
	//pwm16SetDC4(pwm16GetPR() / 4 * 3);	//OC4 output at 75% duty cycle
	return clkgen16Get();
}

//initialize clkgen17 - output pin defined by TIM17CH1toPIN() macro
#define clkgen17Get() (F_PHB / pwm17GetPS() / pwm17GetPR())	//return frequency
uint32_t clkgen17Set(uint32_t div) {
	pwm17Init(1); pwm17SetPR(div / pwm17GetPS());			//initialize clock, set prescaler
	pwm17SetDC1(pwm17GetPR() / 4 * 2);	//OC1 output at 50% duty cycle
	//pwm17SetDC2(pwm17GetPR() / 4 * 1);	//OC2 output at 25% duty cycle
	//pwm17SetDC3(pwm17GetPR() / 4 * 2);	//OC3 output at 50% duty cycle
	//pwm17SetDC4(pwm17GetPR() / 4 * 3);	//OC4 output at 75% duty cycle
	return clkgen17Get();
}

//generator clock signals
void clkgen(void) {
#if defined(CLKGEN0_DIV)
	clkgen0Set(CLKGEN0_DIV);
#endif

#if defined(CLKGEN1_DIV)
	clkgen1Set(CLKGEN1_DIV);
#endif

#if defined(CLKGEN2_DIV)
	clkgen2Set(CLKGEN2_DIV);
#endif

#if defined(CLKGEN3_DIV)
	clkgen3Set(CLKGEN3_DIV);
#endif

#if defined(CLKGEN14_DIV)
	clkgen14Set(CLKGEN14_DIV);
#endif

#if defined(CLKGEN16_DIV)
	clkgen16Set(CLKGEN16_DIV);
#endif

#if defined(CLKGEN17_DIV)
	clkgen17Set(CLKGEN17_DIV);
#endif
}

//flip the led
void led_flp(void) {
	pinFlip(LED);
}

//user defined set up code
void setup(void) {
	//configure the system clock
	//SystemCoreClockHSI();						//hsi as clock
	//SystemCoreClockHSIPLL();					//hsixpll as clock
	//SystemCoreClockHSIPLL4M();

	//configure pins
#if defined(VDD)
	pinMode(VDD, OUTPUT); digitalWrite(VDD, HIGH);
#endif
#if defined(GND)
	pinMode(GND, OUTPUT); digitalWrite(GND, LOW);
#endif
	pinMode(LED, OUTPUT);						//led as output pin

	//initialize the uart
	//uart1Init(UART_BR115K2);					//initialize uart1
	uart2Init(UART_BR115K2);					//initialize uart2
	//lpuart1Init(UART_BR115K2);				//initialize lpuart1

	//tmr initialization
	//tmr2Init(1); 
	//tmr2OC1SetPR(10000); tmr2OC1AttachISR(led_flp);
	//tmr2OC2SetPR(10100); tmr2OC2AttachISR(led_flp);
	//tmr3Init(10); 
	//tmr3OC1SetPR(10000); tmr3OC1AttachISR(led_flp);
	//tmr3OC2SetPR(10100); tmr3OC2AttachISR(led_flp);
	//lptmr2Init(LPTIM_PS64x); /*lptmr2AttachISR(led_flp); */lptmr2Start();
	//tmr16Init(1000); 
	//tmr31Init(1);
	
	//pwm
	//pwm16Init(1); pwm16SetDC1(10); 

	//initialize pins to AFIO
	//pin2AFIO2(PB13); 
	//pin2AFIO(PA10, AFIO3);

	//reset the rtc
	//RTCInit(RCC_BDCR_RTCSEL_LSI, F_LSI); time2RTC(1234567890ul);
	//sRTCInit(); time2sRTC(RTC2time(NULL));

	//initialize rng
	//rngInit();
	
	//aesInit();
	
	//initialize clock gen
	clkgen();
	
	ei();										//enable interrupts	
}

//user defined main loop
void loop(void) {
	uint32_t tmp0, tmp;
	//static uint32_t sec0, sec1;
	static uint32_t tick0=0;

	//update software rtc
	//sRTCUpdate();
	
	
	//if (tmr2Get() - tick0 > LED_DLY) {tick0 += LED_DLY;						//advance to the next match point
	//if (lptmr2OVF()) {lptmr2OVFCLR(); lptmr2Start();	//tick0 += LED_DLY;						//advance to the next match point
	//if (tmr16Get() - tick0 > 10000) {tick0 += 10000;
	//if (tick0++ > LED_DLY / 500) {tick0=0;
	if (ticks() - tick0 > LED_DLY) {tick0 += LED_DLY;						//advance to the next match point
		pinFlip(LED);							//flip led, 105 ticks

		//benchmark
		tmp0 = ticks();							//stamp tick1
		//do something
		//digitalRead(PA5);						//read a pin, 59 ticks (base case = 29 ticks, if there is nothing)
		//uart1Init(UART_BR9600);					//in   itial uart, 1328 ticks
		//digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks
		//tmp=analogRead(ADC_TS);
		//for (tmp=0; tmp<1000; tmp++) digitalWrite(LED, !digitalRead(LED));	//flip led, 48K/1K ticks
		//for (tmp=0; tmp<1000; tmp++) IO_FLP(GPIOB, 1<<7);					//flip led, 11K/1K ticks
		//for (tmp=0; tmp<1000/5; tmp++) {digitalWrite(LED, !digitalRead(LED));digitalWrite(LED, !digitalRead(LED));digitalWrite(LED, !digitalRead(LED));digitalWrite(LED, !digitalRead(LED));digitalWrite(LED, !digitalRead(LED));}	//flip led, 42.1K/1K ticks
		//for (tmp=0; tmp<1000/5; tmp++) {IO_FLP(GPIOB, 1<<7);	IO_FLP(GPIOB, 1<<7);	IO_FLP(GPIOB, 1<<7);	IO_FLP(GPIOB, 1<<7);	IO_FLP(GPIOB, 1<<7);	}				//flip led, 4.6K/1K ticks
		//tmp=ticks() - tmp0;
		tmp0 = ticks() - tmp0;					//calculate time elapsed

		//display something
		//u2Print("F_CORE=                    ", SystemCoreClock);
		u2Print("F_CPU =                    ", F_CPU);
		//u2Print("F_UART=                    ", F_UART);
		//u2Print("F_PCLK=                    ", F_PCLK);
		u2Print("ticks =                    ", ticks());
		//u2Print("T31CNT=                    ", tmr16Get());
		//u2Print("T2CNT =                    ", tmr2Get());
		//u2Print("brr =                      ", USART2->BRR);
		//u2Print("CalRTC=                    ", LSICalRTC(1));
		//u2Print("CalT14=                    ", LSICalT14(4));
		u2Print("u2bps =                    ", u2bps());
		//u2Print("ticks =                    ", tick0);
		//u2Print("tmp0  =                    ", tmp0);
		//u2Print("adc   =                    ", analogRead(ADC_VBG));
		//u2Print("adc_e =                    \r\n", analogRead(ADC_VBG2));
		//u2Print("ticks =                    ", tmp1-tmp0);
		//u2Print("rng   =                    ", rngGet());		
		//lpuart1Puts(aesCipher(str));
		u2Println();

		//rtt
		//SEGGER_RTT_WriteString(0, "###### Testing SEGGER_printf() ######\r\n");
	}
}
