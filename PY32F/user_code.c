//PY32F002A/030 - programmable clock generator / frequency divider
// - up to 6 independent channels of output frequencies (1x MCO + 5x TIMERs)
// - up to 9 Output pins: pin assignment and duty cycles are user programmable
// -   MCO:   PA5
// -   TIM1:  PA3, PB6, PA1
// -   TIM3:  PB0, PB1
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
//PY32duino code
// - using PY32F03002/003/030 chip
// - free running systick for ticks
// - 
// - version history
// - v0.1, 07/02/2023: initial port from stm32f0
// - v0.2, 08/15/2023: implemented gpio, timer, and pwm functionalities
// - v0.2a,08/19/2023: added hardware and software RTC support
// - v0.3, 08/20/2023: implemented PLL
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
//              |                PA4  |>-+--> GND
//              |                     |  |
//       +----->| OSCin/PF0      PA3  |>-+--> LED
//    [Xtal]    |                     |
//       +-----<| OSCout/PF1     PA2  |>----> U2TX
//              |                     |
//              |                     |
//              |                     |//clkgen output pins: see py32duino.h for pin assignment
//              |                     |//all possible output pins listed in bracket
//              |                     |//not all output pins are available on your particular package
//              |                     |
//              |                PA5  |>----> MCO (PA1, PA5, PA8, PA9, PA13, PA14, PF2)
//              |                     |
//              |                PA3  |>----> TIM1OC1 (50% dc) (PA3, PA8)
//              |                     |>----> TIM1OC2 (25% dc) (PA9, PA13)
//              |                PB6  |>----> TIM1OC3 (50% dc) (PA0, PA10, PB6)
//              |                PA1  |>----> TIM1OC4 (75% dc) (PA1)
//              |                     |
//              |                     |>----> TIM3OC1 (50% dc) (PA2, PA6, PB4)
//              |                     |>----> TIM3OC2 (25% dc) (PA5, PA7, PB5)
//              |                PB0  |>----> TIM3OC3 (50% dc) (PB0, PF3)
//              |                PB1  |>----> TIM3OC4 (75% dc) (PB1)
//              |                     |
//              |                PA4  |>----> TIM14OC1 (50% dc) (PA4, PA7, PB1, PF0, PF1)
//              |                PA6  |>----> TIM16OC1 (50% dc) (PA6, PB8)
//              |                PA7  |>----> TIM17OC1 (50% dc) (PA7, PB8)
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |=====================|
//
//
//

#include "py32duino.h"						//we use py32f0

//hardware configuration
#define LED0		PA3						//led pin
#define GND			PA4						//pin grounded
#define LED_DLY		(F_CPU / 2)				//half a second

//for uart debugging
#define VDD			PA1						//pa1 high
#define LEDR		PA2						//led red
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
	pinFlip(LED0);
}

//user defined set up code
void setup(void) {
	//set up the clock - default HSI@8M
	//SystemCoreClockHSI4M();						//select hsi clock
	//SystemCoreClockHSI48M();					//select hsixpll as clock
	//SystemCoreClockHSE();						//systemcoreclock to HSE
	//SystemCoreClockHSIPLL();					//systemcoreclock to HSExpll
	//LSITrimSet(+2);
	
#if defined(GND)
	pinMode(GND, OUTPUT); digitalWrite(GND, LOW);	//gnd pin grounded
#endif
#if defined(VDD)
	pinMode(VDD, OUTPUT); digitalWrite(VDD, HIGH);
#endif
	pinMode(LED0,OUTPUT); digitalWrite(LED0,HIGH);	//led as output pin

	//initialize the uart
	uart1Init(UART_BR115K2);					//initialize uart1
	//uart2Init(UART_BR115K2);					//initialize uart2
	
	//16-bit timers
	//tmr3Init(100); tmr3OC1SetPR(1000); tmr3OC1AttachISR(led_flp); tmr3OC2SetPR(1010); tmr3OC2AttachISR(led_flp);
	//tmr16Init(10); tmr16OC1SetPR(1010); tmr16OC1AttachISR(led_flp);
	
	//configure mco
	//MCO2LSI(4);									//map mco to LSI (32768Hz nominal)

	
	//initialize clock gen
	clkgen();
	
	ei();										//enable interrupts
}

//user defined main loop
void loop(void) {
	uint32_t tmp0, tmp;
	static uint32_t tick0=0;

	//if (tmr16Get() - tick16 > LED_DLY / tmr16GetPS()) {tick16 += LED_DLY / tmr16GetPS();
	//if (tick0++ > 10000ul) { tick0 = 0;						//reset tick0
	if (ticks() - tick0 > LED_DLY) { tick0 += LED_DLY;						//advance to the next match point
		pinFlip(LED0);							//flip led, 105 ticks
		
		//benchmark
		tmp0 = ticks();							//stamp tick1
		//do something
		tmp0 = ticks() - tmp0;					//calculate time elapsed

		//display something
		//u1Print("F_PLL =                    ", F_PLL);
		u1Print("F_CPU =                    ", F_APB);
		u1Print("ticks =                    ", ticks());
		//u1Print("tick0 =                    ", tick0);
		//u1Print("tick16=                    ", tick16);
		//u1Print("tmp0  =                    ", tmp0);
		//u1Print("LSITun=                    ", LSITune2(F_HSI));
		//u1Print("u1brr =                    ", USART1->BRR);
		//u1Print("RTCF  =                    ", LSICalRTC(2));
		//u1Print("T14F  =                    ", LSICalT14(4));
		u1Print("u1bps =                    ", u1bps());
		//u1Print("TIMCLK=                    ", TIMClock());
		//u1Print("T14Tks=                    ", T14Tks(8));
		//u1Print("TSCAL1=                    ", TSCAL1);		//941
		//u1Print("TSCAL2=                    ", TSCAL2);		//1130
		//u1Print("TempF =                    ", ADC2Fx100(analogRead(ADC_TEMP)));
		//u1Print("Vref  =                    ", ADC2mv(analogRead(ADC_VREF)));
		//u1Print("AIN0  =                    ", analogRead(ADC_CH0));
		//u1Print("adc_e=                    \r\n", analogRead(ADC_VBG2));
		//u1Print("ticks=                     ", tmp1-tmp0);
		//u1Print("hRTCSec=                   ", RTC2time(NULL));
		//u1Print("sRTCtim=                   ",sRTC2time(NULL));
		//u1Print("hRTCSec=                   ", RTCTicks(2));		//@trim=0, 4.030M vs. 4M; 
		//u1Print("sRTCSec=                   ",sRTCGetSec());
		//u1Print("d_time =                   ", RTC2time(NULL) - sRTC2time(NULL));
		//u1Print("rtcTks =                   ", rtcTks(4)/4);
		//uart1Puts(asctime(&tmp1));
		u1Println();
		
		//segger rtt
		//SEGGER_RTT_printf(0, "printf Test: %%d,      -12345 : %d.\r\n", tick0);	//1kb in flash
	}
}
