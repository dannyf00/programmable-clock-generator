#include "py32duino.h"					//py32duino

//global defines

//global variables
//declare pins
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
const PIN2GPIO GPIO_PinDef[]={
#if defined(GPIOA)
	{GPIOA, 1<<0},						//STM32duino Pin  0 = RP0/PB0/CHIP PIN4
	{GPIOA, 1<<1},						//STM32duino Pin  1 = RP1/PB1/CHIP PIN5
	{GPIOA, 1<<2},						//STM32duino Pin  2 = RP2/PB2/CHIP PIN6
	{GPIOA, 1<<3},						//STM32duino Pin  3 = RP3/PB3/CHIP PIN7
	{GPIOA, 1<<4},						//STM32duino Pin  4 = RP4/PB4/CHIP PIN11
	{GPIOA, 1<<5},						//STM32duino Pin  5 = RP5/PB5/CHIP PIN14
	{GPIOA, 1<<6},						//STM32duino Pin  6 = RP6/PB6/CHIP PIN15
	{GPIOA, 1<<7},						//STM32duino Pin  7 = RP7/PB7/CHIP PIN16
	{GPIOA, 1<<8},						//STM32duino Pin  8 = RP8/PB8/CHIP PIN17
	{GPIOA, 1<<9},						//STM32duino Pin  9 = RP9/PB9/CHIP PIN18
	{GPIOA, 1<<10},						//STM32duino Pin 10 = RP10/PB10/CHIP PIN21
	{GPIOA, 1<<11},						//STM32duino Pin 11 = RP11/PB11/CHIP PIN22
	{GPIOA, 1<<12},						//STM32duino Pin 12 = RP12/PB12/CHIP PIN23
	{GPIOA, 1<<13},						//STM32duino Pin 13 = RP13/PB13/CHIP PIN24
	{GPIOA, 1<<14},						//STM32duino Pin 14 = RP14/PB14/CHIP PIN25
	{GPIOA, 1<<15},						//STM32duino Pin 15 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOB)
	{GPIOB, 1<<0},						//STM32duino Pin 16 = RP0/PB0/CHIP PIN4
	{GPIOB, 1<<1},						//STM32duino Pin 17 = RP1/PB1/CHIP PIN5
	{GPIOB, 1<<2},						//STM32duino Pin 18 = RP2/PB2/CHIP PIN6
	{GPIOB, 1<<3},						//STM32duino Pin 19 = RP3/PB3/CHIP PIN7
	{GPIOB, 1<<4},						//STM32duino Pin 20 = RP4/PB4/CHIP PIN11
	{GPIOB, 1<<5},						//STM32duino Pin 21 = RP5/PB5/CHIP PIN14
	{GPIOB, 1<<6},						//STM32duino Pin 22 = RP6/PB6/CHIP PIN15
	{GPIOB, 1<<7},						//STM32duino Pin 23 = RP7/PB7/CHIP PIN16
	{GPIOB, 1<<8},						//STM32duino Pin 24 = RP8/PB8/CHIP PIN17
	{GPIOB, 1<<9},						//STM32duino Pin 25 = RP9/PB9/CHIP PIN18
	{GPIOB, 1<<10},						//STM32duino Pin 26 = RP10/PB10/CHIP PIN21
	{GPIOB, 1<<11},						//STM32duino Pin 27 = RP11/PB11/CHIP PIN22
	{GPIOB, 1<<12},						//STM32duino Pin 28 = RP12/PB12/CHIP PIN23
	{GPIOB, 1<<13},						//STM32duino Pin 29 = RP13/PB13/CHIP PIN24
	{GPIOB, 1<<14},						//STM32duino Pin 30 = RP14/PB14/CHIP PIN25
	{GPIOB, 1<<15},						//STM32duino Pin 31 = RP15/PB15/CHIP PIN26
#endif

#if defined(GPIOC)
	{GPIOC, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOC, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOC, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOC, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOC, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOC, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOC, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOC, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOC, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOC, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOC, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOC, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOC, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOC, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOC, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOC, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOC

#if defined(GPIOD)						//pins 48..63
	{GPIOD, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOD, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOD, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOD, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOD, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOD, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOD, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOD, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOD, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOD, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOD, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOD, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOD, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOD, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOD, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOD, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOD

#if defined(GPIOE)						//pins 64..79
	{GPIOE, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOE, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOE, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOE, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOE, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOE, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOE, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOE, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOE, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOE, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOE, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOE, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOE, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOE, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOE, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOE, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOE

#if defined(GPIOF)						//pins 80..96
	{GPIOF, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOF, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOF, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOF, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOF, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOF, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOF, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOF, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOF, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOF, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOF, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOF, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOF, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOF, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOF, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOF, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOF

#if defined(GPIOG)						//pins 80..96
	{GPIOG, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOG, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOG, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOG, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOG, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOG, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOG, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOG, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOG, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOG, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOG, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOG, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOG, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOG, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOG, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOG, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOG
};

//set up core timer
//global variables

//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;
//static uint16_t timer1_fract = 0;

//timer1 overflow isr
//void _ISR_PSV _T2Interrupt(void) {
//	IFS0bits.T2IF=0;					//clear tmr1 interrupt flag
//	timer_ticks+=0x10000ul;				//increment overflow count: 16-bit timer
//}

//configure gpio DDR mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * GPIOx, uint32_t mask, uint32_t mode) {
	uint8_t pos=0, posx2;
	//uint32_t pin_mask;

	while (mask) {								//for (pos=0; pos < 16; pos++) {
		//looking for pin position / mask
		//pin_mask = 1ul << pos;
		if (mask & (1ul << 0)) {				//if (mask & (1ul << pos)) {
			posx2=pos * 2;
			//we have found the pos / pin_mask
			if ((mode & GPIOMODE_OUTPUT) || (mode & GPIOMODE_AF)) {
				GPIOx->OSPEEDR &=~(0x03ul << (posx2));	//clear ospeeder
				GPIOx->OSPEEDR |= (0x01ul << (posx2));	//set to medium speed (0x01)

				GPIOx->OTYPER &=~(1ul << pos);				//clear otyper
				GPIOx->OTYPER |= ((mode & GPIOMODE_PP)?0ul:1ul) << pos;	//0->pp, 1->od
			}

			GPIOx->MODER &=~(0x03 << (2 * pos));			//clear moder
			GPIOx->MODER |= (mode & 0x03) << (posx2);		//set moder

			GPIOx->PUPDR &=~(0x03 << (posx2));			//clear pupdr
			GPIOx->PUPDR |= ((mode >> 4) & 0x03) << (posx2);	//set pupdr
		}
		mask = mask >> 1;
		pos = pos + 1;
	}
}

//empty interrupt handler
void empty_handler(void) {
	//do nothing here
}

//global variables
//for time base off SysTick (24-bit counter)
volatile uint32_t systick_ovf = 1ul<<24;						//time base on Systick -> SysTick->VAL being the lowest 24-bits (SysTick is a downcounter)
static void (* _systick_isrptr)(void)=empty_handler;				//systick_ptr pointing to empty_handler by default


//systick handler - to provide time base for millis()/micros()
void SysTick_Handler(void) {
	//clear the flag
	systick_ovf += 1ul<<24;						//increment systick counter - 24bit, 1:1 prescaler
	_systick_isrptr();									//execute systick isr
}

//install systick isr
//activate the isr handler
void systickAttachISR(void (*isrptr)(void)) {
	_systick_isrptr = isrptr;
}

//new code
//clock mgmt
//switch system clock
//clock choices: RCC_CFGR_SW_HSI, RCC_CFGR_SW_HSE, RCC_CFGR_SW_PLL (from HSI or HSE)
void SystemCoreClockConfig(uint32_t clk) {
	switch (clk) {
		case RCC_CFGR_SW_HSI: RCC->CR  |= RCC_CR_HSION;   while ((RCC->CR  & RCC_CR_HSIRDY  )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on HSI and wait for it to stablize
		case RCC_CFGR_SW_HSE: RCC->CR  |= RCC_CR_HSEON;   while ((RCC->CR  & RCC_CR_HSERDY  )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on HSe and wait for it to stablizebreak;
#if defined(RCC_CFGR_SWS_PLL)
		case RCC_CFGR_SW_PLL: RCC->CR  |= RCC_CR_PLLON;   while ((RCC->CR  & RCC_CR_PLLRDY  )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on PLL and wait for it to stablizebreak;
#endif	//pll
		case RCC_CFGR_SW_LSI: RCC->CSR |= RCC_CSR_LSION;  while ((RCC->CSR & RCC_CSR_LSIRDY )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on LSI and wait for it to stablizebreak;
#if defined(RCC_CFGR_SWS_LSE)
		case RCC_CFGR_SW_LSE: RCC->BDCR|= RCC_BDCR_LSEON; while ((RCC->BDCR& RCC_BDCR_LSERDY)== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on LSE and wait for it to stablizebreak;
#endif	//lse
	}
	SystemCoreClockUpdate();								//update the clock
}

#if !defined(PY32F002Ax5)
//set pll
//fixed 2x
void pllSet(void) {
	FLASH->ACR = (FLASH->ACR & FLASH_ACR_LATENCY) | (1<<FLASH_ACR_LATENCY_Pos);	//1->above 24M, 0->at or below 24M

	//switch to HSI to be safe
	//RCC->CR |= RCC_CR_HSION; while ((RCC->CR & RCC_CR_HSIRDY) == 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | RCC_CFGR_SW_HSI; 
	//enable pll
	RCC->PLLCFGR &=~RCC_PLLCFGR_PLLSRC;	//0->HSI, 1->HSE
	//do {RCC->CR |= RCC_CR_PLLON;} while ((RCC->CR & RCC_CR_PLLRDY) == 0);
	RCC->CR |= RCC_CR_PLLON; while ((RCC->CR & RCC_CR_PLLRDY) == 0) continue; 				//wait for rcc to be powered down
	
	//readjust flash latency if clock is less than 24Mhz
	if (AHBClock() < 24000000ul) FLASH->ACR = (FLASH->ACR & FLASH_ACR_LATENCY) | (0<<FLASH_ACR_LATENCY_Pos);	//1->above 24M, 0->below 24M
}

//get pll speed
uint32_t PLLClock(void) {
	uint32_t tmp=0;
	if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) tmp = F_HSE; else tmp = F_HSI;
	return 2*tmp;					//pll fixed at 2x
}
#endif

//set hsi frequency
//hsi_fs = HSI_FS24, HSI_FS22, HSI_FS16, HSI_FS8, HSI_FS4
uint32_t F_HSISet(uint32_t f_hsi) {
	uint32_t tmp;
	
	switch (f_hsi) {
		case HSI_FS24: tmp = HSI_FS24 | ((HSI24TRIM << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM); break;
		case HSI_FS22: tmp = HSI_FS22 | ((HSI22TRIM << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM); break;
		case HSI_FS16: tmp = HSI_FS16 | ((HSI16TRIM << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM); break;
		case HSI_FS8 : tmp = HSI_FS8  | ((HSI8TRIM  << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM); break;
		case HSI_FS4 : tmp = HSI_FS4  | ((HSI4TRIM  << RCC_ICSCR_HSI_TRIM_Pos) & RCC_ICSCR_HSI_TRIM); break;
		default: f_hsi=0;
	}
	RCC->ICSCR = (RCC->ICSCR &~(RCC_ICSCR_HSI_FS | RCC_ICSCR_HSI_TRIM)) | tmp;
	return f_hsi;
}

//get hsi frequency
//hsi_fs = HSI_FS24, HSI_FS22, HSI_FS16, HSI_FS8, HSI_FS4
uint32_t F_HSIGet(void) {
	uint32_t tmp;
	
	switch (RCC->ICSCR & RCC_ICSCR_HSI_FS) {
		case HSI_FS24: tmp = F_HSI24; break;
		case HSI_FS22: tmp = F_HSI22; break;
		case HSI_FS16: tmp = F_HSI16; break;
		case HSI_FS8 : tmp = F_HSI8 ; break;
		case HSI_FS4 : tmp = F_HSI4 ; break;
		default: tmp=0;
	}
	return tmp;
}

//hsi_fs = HSI_FS24, HSI_FS22, HSI_FS16, HSI_FS8, HSI_FS4
//Get factory HSITrim
uint32_t HSITrimGetF(void) {
	uint32_t tmp;
	
	switch (RCC->ICSCR & RCC_ICSCR_HSI_FS) {
		case HSI_FS24: tmp = HSI24TRIM; break;
		case HSI_FS22: tmp = HSI22TRIM; break;
		case HSI_FS16: tmp = HSI16TRIM; break;
		case HSI_FS8 : tmp = HSI8TRIM ; break;
		case HSI_FS4 : tmp = HSI4TRIM ; break;
		default: tmp=0;
	}
	return tmp;
}

//get SYSCLK speed, just beofre AHBDIV
uint32_t SYSClock(void) {
	uint32_t tmp=0;
	switch (RCC->CFGR & RCC_CFGR_SWS) {
		case RCC_CFGR_SWS_HSI: tmp = F_HSI; break;
		case RCC_CFGR_SWS_HSE: tmp = F_HSE; break;
#if !defined(PY32F002Ax5)
		case RCC_CFGR_SWS_PLL: tmp = F_PLL; break;
#endif
		case RCC_CFGR_SWS_LSI: tmp = F_LSI; break;
#if !defined(PY32F002Ax5)
		case RCC_CFGR_SWS_LSE: tmp = F_LSE; break;
#endif
	}
	return tmp;
}

//get AHBClock, after AHBDIV
uint32_t AHBClock(void) {
	//uint32_t tmp;

	return SYSClock() >> ((RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos);
}

//get APB speed
uint32_t APBClock(void) {
	//uint32_t tmp;
	return AHBClock() >> ((RCC->CFGR & RCC_CFGR_PPRE_Msk) >> RCC_CFGR_PPRE_Pos);								//get AHB speed
}

//get TIM_PCLK speed
uint32_t TIMClock(void) {
	return ((RCC->CFGR & RCC_CFGR_PPRE_Msk) == RCC_CFGR_PPRE_0)?APBClock():(2*APBClock());
}

//uart1 speed = APBClock()
uint32_t u1FreqGet(void) {
	return APBClock();
}


//reset the mcu
void mcuInit(void) {
	//waste time - for debugging
	volatile uint32_t i=100000ul; while (i--) continue;
	
	//set up the clock - default hsi@8M
	//SystemCoreClockHSI();						//systemcoreclock to HSI - default 8M
	//SystemCoreClockHSIPLL();					//systemcoreclock to HSI
	//SystemCoreClockHSE();						//systemcoreclock to HSE
	//SystemCoreClockHSEPLL();					//systemcoreclock to HSExpll
	//SystemCoreClockHSI4M();					//select hsi clock
	//SystemCoreClockHSI22MPLL();				//select hsixpll as clock

	//enable clock to GPIO
	RCC->IOPENR |=
#if defined(GPIOA)
			RCC_IOPENR_GPIOAEN |
#endif
#if defined(GPIOB)
			RCC_IOPENR_GPIOBEN |
#endif
#if defined(GPIOC)
			RCC_IOPENR_GPIOCEN |
#endif
#if defined(GPIOD)
			RCC_IOPENR_GPIODEN |
#endif
#if defined(GPIOE)
			RCC_IOPENR_GPIOEEN |
#endif
#if defined(GPIOF)
			RCC_IOPENR_GPIOFEN |
#endif
#if defined(GPIOG)
			RCC_IOPENR_GPIOGEN |
#endif
			0x00;
	
	//start systick / coretick
	//configure Systick as the time base for millis()/micros()

	systick_ovf = 1ul<<24;											//SysTick is a 24-bit downcounter
	//for chips where SysTick_Config() is not defined in cmsis
	SysTick->LOAD  = 	(systick_ovf-1)/*ticks*/ & SysTick_LOAD_RELOAD_Msk;      /* set reload register */
	SysTick->VAL   = 	0;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL  = 	SysTick_CTRL_CLKSOURCE_Msk |
						SysTick_CTRL_TICKINT_Msk   |
						SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
	NVIC_SetPriority 	(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
	_systick_isrptr = empty_handler;
	//alternative - for CMSIS-equip'd chips
	//SysTick_Config(SysTick_LOAD_RELOAD_Msk);			//reload all 24 bits

	//enable global interrupts
	ei();										//testing

	//update SystemCoreClock - it must be the last step before exiting mcuInit
	SystemCoreClockUpdate();
}

//return timer clicks
//execution time = 30 ticks, no optimization
uint32_t systicks(void) {
	volatile uint32_t m;
	volatile uint32_t f;

	//do a double read
	do {
		m = systick_ovf;			//read the overflow counter
		f = SysTick->VAL;				//read the least significant 16-bits
	} while (m != systick_ovf);	//guard against overflow

	return (m - f) << 0;				//systick is a 24-bit downcounter
}

//C main loop
int main(void) {

	mcuInit();						//reset the mcu
	setup();						//run the setup code
	while (1) {
		loop();						//run the default loop
	}
}


//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
#define inline						//supress inline decalration
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;
	
	//if (mode==OUTPUT) GIO_OUT(gpio, mask); else GIO_IN(gpio, mask);
	switch (mode) {
		case OUTPUT: 		GIO_OUTPP(gpio, mask); break;
		case OUTPUT_OD: 	GIO_OUTOD(gpio, mask); break;
		case INPUT: 		GIO_INFL(gpio, mask); break;
		case INPUT_PU: 		GIO_INPU(gpio, mask); break;
		case INPUT_PD: 		GIO_INPD(gpio, mask); break;
	}
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;
	
	if (val==LOW) GIO_CLR(gpio, mask); else GIO_SET(gpio, mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;

	return (GIO_GET(gpio, mask))?HIGH:LOW;
}
//Arduino Functions: GPIO
//helper functions
uint32_t pinPos(uint32_t mask) {
	uint32_t tmp=0;
	while ((mask & (1ul<<tmp)) == 0) tmp++;
	return tmp;
}

//configure pin for AFIO: AFIOn:[0..15]
void pinAFIO(PIN_TypeDef pin, AFIO_TypeDef AFIOn) {
	uint32_t pos;
	GPIO_TypeDef *gpio;

	gpio=GPIO_PinDef[pin].gpio;
	//mask=GPIO_PinDef[pin].mask;
	pos = pinPos(GPIO_PinDef[pin].mask);
	gpio->MODER = (gpio->MODER &~(/*0b11*/0x03<<(2* pos))) | (/*0b10*/0x02<<(2* pos));	//0x02->AFIO
	gpio->OTYPER &=~(1<< pos);	//0->default
	gpio->OSPEEDR = (gpio->OSPEEDR &~(/*0b11*/0x03<<(2* pos))) | (/*0b01*/0x01<<(2* pos));	//0x01->low speed
	if (pos<8) gpio->AFR[0] = (gpio->AFR[0] &~(0x0f<<(4*( pos%8)))) | ((AFIOn-AFIO0)<<(4*( pos%8)));
	else gpio->AFR[1] = (gpio->AFR[1] &~(0x0f<<(4*( pos%8)))) | ((AFIOn-AFIO0)<<(4*( pos%8)));
}
//end GPIO

//ticks()
//Arduino Functions: Time
//return timer ticks
	
//delay millisseconds
//void delay(uint32_t ms) {
//	uint32_t start_time = ticks();
//	ms *= cyclesPerMillisecond();
//	while (ticks() - start_time < ms) continue;
//}

//delay micros seconds
//void delayMicroseconds(uint32_t us) {
//	uint32_t start_time = ticks();
//	us *= cyclesPerMicrosecond();
//	while (ticks() - start_time < us) continue;
//}

//delay ticks
void delayTks(uint32_t tks) {
	uint32_t start_time = ticks();
	while (ticks() - start_time < tks) continue;
}

//end Time

//generic uart reset routine - to save space
void uartInit(USART_TypeDef *USARTx, unsigned long baudrate) {
	//uint32_t uartdiv;

	USARTx->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USARTx->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16, 1->oversampling by 8
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USARTx->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USARTx->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    USARTx->BRR = (F_UART + baudrate / 2) / baudrate;						//without rounding: F_UART / baudrate
    //USARTx->BRR /= (USARTx->CR1 & (1<<15))?2:1;		//per datasheet, for OVER8=0 or 1

    //USARTx->DR = 0;					//reset the data register
    //USARTx->SR = 0;					//reset the data register

    //enable uart1
    USARTx->CR1 |= USART_CR1_UE;		//'0'->disable uart, '1'->enable uart

}

//generic print to uart buffer
void uPrint(char *uRAM, int32_t dat) {
	uint8_t idx=20;						//buffer index
	uint8_t negFlag=0;					//flag for negative sign: 1->negative, 0->positive

	if (dat<0) {negFlag = 1; dat = -dat;}
#if 1	//36k/invocation
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
#else	//36k/invocation
	uint32_t tmp;

	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	                      uRAM[idx--]=',';
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	                      uRAM[idx--]=',';
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
	                      uRAM[idx--]=',';
	tmp = dat; dat /= 10; uRAM[idx--]='0'+(tmp - dat * 10);
#endif
	uRAM[idx--]=negFlag?'-':' ';
}

//uart1
//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart1Init(unsigned long baudrate) {
	//uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

	//reset usart
	uartInit(USART1, baudrate);

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    //IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    //IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));
#if defined(U1TX2PIN)
    U1TX2PIN();
#endif

#if defined(U1RX2PIN)
    U1RX2PIN();
#endif
}

void uart1Putch(char ch) {
    //while (!(USART1->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
    while (uart1Busy()) continue;    			//wait for the transmission buffer to be empty
    USART1->DR = ch;                        	//load the data buffer
    //while (!(USART1->SR & (1<<6)));    		//wait for the transmission to complete
}

//put a string
void uart1Puts(char *str) {
	while (*str) uart1Putch(*str++);
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

//put a line termined with ln
void uart1Putline(char *ln) {
	//USARTWriteString(ln);
	uart1Puts(ln);
	//USARTWriteString("\r\n");
	uart1Puts((char *)"\r\n");
}

//get the received char
uint8_t uart1Getch(void) {
	   //while (!(USART1->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
	    return USART1->DR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart1Available(void) {
	//return (USART1->SR & USART_SR_TC)?true:false;
	return (USART1->SR & USART_SR_RXNE);
}

//test if uart tx is busy
uint16_t uart1Busy(void) {
    return !(USART1->SR & USART_SR_TXE);    	//return 1 if TX buffer is empty
}

//print to uart1
void u1Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	uPrint(uRAM, dat);					//print to uRAM
	uart1Puts(uRAM);					//send a message on uart1
}	

#if defined(USART2)
//uart2
//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart2Init(unsigned long baudrate) {
	//uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APBENR1 |= RCC_APBENR1_USART2EN;

	//reset usart
	uartInit(USART2, baudrate);

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    //IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    //IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));
#if defined(U2TX2PIN)
    U2TX2PIN();
#endif
#if defined(U2RX2PIN)
    U2RX2PIN();
#endif


}

void uart2Putch(char ch) {
	   //while (!(USART2->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
	    while (uart2Busy()) continue;    			//wait for the transmission buffer to be empty
	    USART2->DR = ch;                        	//load the data buffer
	    //while (!(USART2->SR & (1<<6)));    		//wait for the transmission to complete
}

void uart2Puts(char *str) {
	while (*str) uart2Putch(*str++);
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

void uart2Putline(char *ln) {
	//USARTWriteString(ln);
	uart2Puts(ln);
	//USARTWriteString("\r\n");
	uart2Puts((char *)"\r\n");
}

uint8_t uart2Getch(void) {
	   //while (!(USART2->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
	    return USART2->DR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart2Available(void) {
	//return (USART2->SR & USART_SR_TC)?true:false;
	return (USART2->SR & USART_SR_RXNE);
}

//test if uart tx is busy
uint16_t uart2Busy(void) {
	   return !(USART2->SR & USART_SR_TXE);    	//return 1 if TX buffer is empty
}

//print to uart2
void u2Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	uPrint(uRAM, dat);					//print to uRAM
	uart2Puts(uRAM);					//send a message on uart1
}	
#endif		//USART2
//end Serial

//timer generic reset routine, to save space
void tmrInit(TIM_TypeDef *TIMx, uint16_t ps) {
	//source from internal clock -> disable slave mode
	TIMx->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIMx->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIMx->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIMx->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIMx->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIMx->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIMx->SR &=~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIMx->DIER &=~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIMx->PSC = ps - 1;			//set the prescaler
	TIMx->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIMx->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIMx->CNT = 0;						//reset the counter
	TIMx->CCR1= TIMx->CCR2= TIMx->CCR3= TIMx->CCR4= 0;				//reset CCRn

	//enable the timer.
	TIMx->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//tmr1
//global variables
static void (* _tim1_ovfisrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_cc1isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_cc2isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_cc3isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_cc4isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default

static uint16_t _tim1_cc1=0;				//output compare registers
static uint16_t _tim1_cc2=0;
static uint16_t _tim1_cc3=0;
static uint16_t _tim1_cc4=0;

//isr for timer1 ovf
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
	//tmr1 ovf isr
	if (TIM1->SR & TIM_SR_UIF) {TIM1->SR &=~TIM_SR_UIF; _tim1_ovfisrptr();}
}


//isr for timer1 capture / compare
void TIM1_CC_IRQHandler(void) {
	//oc1..4 portion
	//output compare 1 flag is set
	//clear the flag, update the output compare register, execute user handler
	if ((TIM1->SR & TIM_SR_CC1IF) && (TIM1->DIER & TIM_DIER_CC1IE)) {TIM1->SR &=~TIM_SR_CC1IF; if (TIM1->CCER & TIM_CCER_CC1E) _tim1_cc1 = TIM1->CCR1; else TIM1->CCR1 += _tim1_cc1; _tim1_cc1isrptr();}
	if ((TIM1->SR & TIM_SR_CC2IF) && (TIM1->DIER & TIM_DIER_CC2IE)) {TIM1->SR &=~TIM_SR_CC2IF; if (TIM1->CCER & TIM_CCER_CC2E) _tim1_cc2 = TIM1->CCR2; else TIM1->CCR2 += _tim1_cc2; _tim1_cc2isrptr();}
	if ((TIM1->SR & TIM_SR_CC3IF) && (TIM1->DIER & TIM_DIER_CC3IE)) {TIM1->SR &=~TIM_SR_CC3IF; if (TIM1->CCER & TIM_CCER_CC3E) _tim1_cc3 = TIM1->CCR3; else TIM1->CCR3 += _tim1_cc3; _tim1_cc3isrptr();}
	if ((TIM1->SR & TIM_SR_CC4IF) && (TIM1->DIER & TIM_DIER_CC4IE)) {TIM1->SR &=~TIM_SR_CC4IF; if (TIM1->CCER & TIM_CCER_CC4E) _tim1_cc4 = TIM1->CCR4; else TIM1->CCR4 += _tim1_cc4; _tim1_cc4isrptr();}
}

//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

	//reset timer
	tmrInit(TIM1, ps);
}


//activate the ovf isr handler
void tmr1OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);		//disable irq

	_tim1_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_UIF;				//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_UIE;			//enable the isr

	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1OC1SetPR(uint16_t pr) {
	//save the period value
	_tim1_cc1 = pr - 0;
	TIM1->CCR1 = _tim1_cc1;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1OC2SetPR(uint16_t pr) {
	//save the period value
	_tim1_cc2 = pr - 0;
	TIM1->CCR2 = _tim1_cc2;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1OC3SetPR(uint16_t pr) {
	//save the period value
	_tim1_cc3 = pr - 0;
	TIM1->CCR3 = _tim1_cc3;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1OC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1OC4SetPR(uint16_t pr) {
	//save the period value
	_tim1_cc4 = pr - 0;
	TIM1->CCR4 = _tim1_cc4;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1OC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr1IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM1->CCMR1 = (TIM1->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM1->CCMR1 = (TIM1->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM1->SR &=~TIM_SR_CC1IF; TIM1->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM1->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM1 rmp to MCO (=3)
	//TIM1->OR = (TIM1->OR &~TIM1_OR_TI1_RMP) | (((3) << 0) & TIM1_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM1->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM1CH1toPIN)
	TIM1CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr1IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.

}

//ic2
void tmr1IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM1->CCMR1 = (TIM1->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM1->CCMR1 = (TIM1->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM1->SR &=~TIM_SR_CC2IF; TIM1->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM1->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM1 rmp to MCO (=3)
	//TIM1->OR = (TIM1->OR &~TIM1_OR_TI1_RMP) | (((3) << 0) & TIM1_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM1->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM1CH2toPIN)
	TIM1CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr1IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.

}

//ic3
void tmr1IC3Init(uint8_t ps) {
	//set up the capture channel 1
	TIM1->CCMR2 = (TIM1->CCMR2 &~TIM_CCMR2_CC3S) | (TIM_CCMR2_CC3S_0);	//0b01->input capture
	TIM1->CCMR2 = (TIM1->CCMR2 &~TIM_CCMR2_IC3PSC) | ((ps<<TIM_CCMR2_IC3PSC_Pos) & TIM_CCMR2_IC3PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM1->SR &=~TIM_SR_CC3IF; TIM1->DIER &=~TIM_DIER_CC3IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM1->CCER |= TIM_CCER_CC3E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM1 rmp to MCO (=3)
	//TIM1->OR = (TIM1->OR &~TIM1_OR_TI1_RMP) | (((3) << 0) & TIM1_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM1->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM1CH3toPIN)
	TIM1CH3toPIN();											//configure the input capture pin
#endif	//ch3
}

//activate the isr handler
void tmr1IC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic4
void tmr1IC4Init(uint8_t ps) {
	//set up the capture channel 1
	TIM1->CCMR2 = (TIM1->CCMR2 &~TIM_CCMR2_CC4S) | (TIM_CCMR2_CC4S_0);	//0b01->input capture
	TIM1->CCMR2 = (TIM1->CCMR2 &~TIM_CCMR2_IC4PSC) | ((ps<<TIM_CCMR2_IC4PSC_Pos) & TIM_CCMR2_IC4PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM1->SR &=~TIM_SR_CC4IF; TIM1->DIER &=~TIM_DIER_CC4IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM1->CCER |= TIM_CCER_CC4E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM1 rmp to MCO (=3)
	//TIM1->OR = (TIM1->OR &~TIM1_OR_TI1_RMP) | (((3) << 0) & TIM1_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM1->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM1CH4toPIN)
	TIM1CH4toPIN();											//configure the input capture pin
#endif	//ch4
}

//activate the isr handler
void tmr1IC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#if defined(TIM2)						//TIM2 does not exist on PY32
//tmr2
//global variables
static void (* _tim2_ovfisrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc1isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc2isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc3isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc4isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default

static uint16_t _tim2_cc1=0;				//output compare registers
static uint16_t _tim2_cc2=0;
static uint16_t _tim2_cc3=0;
static uint16_t _tim2_cc4=0;

//isr for timer1 capture / compare
void TIM2_IRQHandler(void) {
	//oc1 portion
	if (TIM2->SR & TIM_SR_CC1IF) {TIM2->SR &=~TIM_SR_CC1IF; if (TIM2->CCER & TIM_CCER_CC1E) _tim2_cc1 = TIM2->CCR1; else TIM2->CCR1 += _tim2_cc1; _tim2_cc1isrptr();}
	if (TIM2->SR & TIM_SR_CC2IF) {TIM2->SR &=~TIM_SR_CC2IF; if (TIM2->CCER & TIM_CCER_CC2E) _tim2_cc2 = TIM2->CCR2; else TIM2->CCR2 += _tim2_cc2; _tim2_cc2isrptr();}
	if (TIM2->SR & TIM_SR_CC3IF) {TIM2->SR &=~TIM_SR_CC3IF; if (TIM2->CCER & TIM_CCER_CC3E) _tim2_cc3 = TIM2->CCR3; else TIM2->CCR3 += _tim2_cc3; _tim2_cc3isrptr();}
	if (TIM2->SR & TIM_SR_CC4IF) {TIM2->SR &=~TIM_SR_CC4IF; if (TIM2->CCER & TIM_CCER_CC4E) _tim2_cc4 = TIM2->CCR4; else TIM2->CCR4 += _tim2_cc4; _tim2_cc4isrptr();}

	//overflow isr
	if (TIM2->SR & TIM_SR_UIF) {TIM2->SR &=~TIM_SR_UIF; _tim2_ovfisrptr();}
}

//initialize the timer1 (16bit)
void tmr2Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	//reset timer
	tmrInit(TIM2, ps);
}

//activate the isr handler
void tmr2OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);			//disable irq

	_tim2_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM2_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC1SetPR(uint16_t pr) {
	//save the period value
	_tim2_cc1 = pr - 0;
	TIM2->CCR1 = _tim2_cc1;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr2OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);			//disable irq

	_tim2_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM2_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC2SetPR(uint16_t pr) {
	//save the period value
	_tim2_cc2 = pr - 0;
	TIM2->CCR2 = _tim2_cc2;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr2OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);			//disable irq

	_tim2_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM2_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC3SetPR(uint16_t pr) {
	//save the period value
	_tim2_cc3 = pr - 0;
	TIM2->CCR3 = _tim2_cc3;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr2OC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);			//disable irq

	_tim2_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM2_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC4SetPR(uint16_t pr) {
	//save the period value
	_tim2_cc4 = pr - 0;
	TIM2->CCR4 = _tim2_cc4;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr2OC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);			//disable irq

	_tim2_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr2IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM2->CCMR1 = (TIM2->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM2->CCMR1 = (TIM2->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM2->SR &=~TIM_SR_CC1IF; TIM2->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM2->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM2 rmp to MCO (=3)
	//TIM2->OR = (TIM2->OR &~TIM2_OR_TI1_RMP) | (((3) << 0) & TIM2_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM2->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM2CH1toPIN)
	TIM2CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr2IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr2IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM2->CCMR1 = (TIM2->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM2->CCMR1 = (TIM2->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM2->SR &=~TIM_SR_CC2IF; TIM2->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM2->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM2 rmp to MCO (=3)
	//TIM2->OR = (TIM2->OR &~TIM2_OR_TI1_RMP) | (((3) << 0) & TIM2_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM2->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM2CH2toPIN)
	TIM2CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr2IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic3
void tmr2IC3Init(uint8_t ps) {
	//set up the capture channel 1
	TIM2->CCMR2 = (TIM2->CCMR2 &~TIM_CCMR2_CC3S) | (TIM_CCMR2_CC3S_0);	//0b01->input capture
	TIM2->CCMR2 = (TIM2->CCMR2 &~TIM_CCMR2_IC3PSC) | ((ps<<TIM_CCMR2_IC3PSC_Pos) & TIM_CCMR2_IC3PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM2->SR &=~TIM_SR_CC3IF; TIM2->DIER &=~TIM_DIER_CC3IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM2->CCER |= TIM_CCER_CC3E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM2 rmp to MCO (=3)
	//TIM2->OR = (TIM2->OR &~TIM2_OR_TI1_RMP) | (((3) << 0) & TIM2_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM2->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM2CH3toPIN)
	TIM2CH3toPIN();											//configure the input capture pin
#endif	//ch3
}

//activate the isr handler
void tmr2IC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic4
void tmr2IC4Init(uint8_t ps) {
	//set up the capture channel 1
	TIM2->CCMR2 = (TIM2->CCMR2 &~TIM_CCMR2_CC4S) | (TIM_CCMR2_CC4S_0);	//0b01->input capture
	TIM2->CCMR2 = (TIM2->CCMR2 &~TIM_CCMR2_IC4PSC) | ((ps<<TIM_CCMR2_IC4PSC_Pos) & TIM_CCMR2_IC4PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM2->SR &=~TIM_SR_CC4IF; TIM2->DIER &=~TIM_DIER_CC4IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM2->CCER |= TIM_CCER_CC4E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM2 rmp to MCO (=3)
	//TIM2->OR = (TIM2->OR &~TIM2_OR_TI1_RMP) | (((3) << 0) & TIM2_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM2->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM2CH4toPIN)
	TIM2CH4toPIN();											//configure the input capture pin
#endif	//ch4
}

//activate the isr handler
void tmr2IC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif	//tim2

#if defined(TIM3)
//tmr3
//global variables
static void (* _tim3_ovfisrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_cc1isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_cc2isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_cc3isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_cc4isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default

static uint16_t _tim3_cc1=0;				//output compare registers
static uint16_t _tim3_cc2=0;
static uint16_t _tim3_cc3=0;
static uint16_t _tim3_cc4=0;

//isr for timer1 capture / compare
void TIM3_IRQHandler(void) {
	//oc1 portion
	if ((TIM3->SR & TIM_SR_CC1IF) && (TIM3->DIER & TIM_DIER_CC1IE)) {TIM3->SR &=~TIM_SR_CC1IF; if (TIM3->CCER & TIM_CCER_CC1E) _tim3_cc1 = TIM3->CCR1; else TIM3->CCR1 += _tim3_cc1; _tim3_cc1isrptr();}
	if ((TIM3->SR & TIM_SR_CC2IF) && (TIM3->DIER & TIM_DIER_CC2IE)) {TIM3->SR &=~TIM_SR_CC2IF; if (TIM3->CCER & TIM_CCER_CC2E) _tim3_cc2 = TIM3->CCR2; else TIM3->CCR2 += _tim3_cc2; _tim3_cc2isrptr();}
	if ((TIM3->SR & TIM_SR_CC3IF) && (TIM3->DIER & TIM_DIER_CC3IE)) {TIM3->SR &=~TIM_SR_CC3IF; if (TIM3->CCER & TIM_CCER_CC3E) _tim3_cc3 = TIM3->CCR3; else TIM3->CCR3 += _tim3_cc3; _tim3_cc3isrptr();}
	if ((TIM3->SR & TIM_SR_CC4IF) && (TIM3->DIER & TIM_DIER_CC4IE)) {TIM3->SR &=~TIM_SR_CC4IF; if (TIM3->CCER & TIM_CCER_CC4E) _tim3_cc4 = TIM3->CCR4; else TIM3->CCR4 += _tim3_cc4; _tim3_cc4isrptr();}

	//overflow isr
	if ((TIM3->SR & TIM_SR_UIF) && (TIM3->DIER & TIM_DIER_UIE)) {TIM3->SR &=~TIM_SR_UIF; _tim3_ovfisrptr();}
}

//initialize the timer1 (16bit)
void tmr3Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM3EN;

	//reset timer
	tmrInit(TIM3, ps);
}

//activate the isr handler
void tmr3OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC1SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc1 = pr - 0;
	TIM3->CCR1 = _tim3_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC2SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc2 = pr - 0;
	TIM3->CCR2 = _tim3_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC3SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc3 = pr - 0;
	TIM3->CCR3 = _tim3_cc3;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3OC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC4SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc4 = pr - 0;
	TIM3->CCR4 = _tim3_cc4;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3OC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr3IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM3->CCMR1 = (TIM3->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM3->CCMR1 = (TIM3->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM3->SR &=~TIM_SR_CC1IF; TIM3->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM3->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM3 rmp to MCO (=3)
	//TIM3->OR = (TIM3->OR &~TIM3_OR_TI1_RMP) | (((3) << 0) & TIM3_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM3->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM3CH1toPIN)
	TIM3CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr3IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr3IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM3->CCMR1 = (TIM3->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM3->CCMR1 = (TIM3->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM3->SR &=~TIM_SR_CC2IF; TIM3->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM3->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM3 rmp to MCO (=3)
	//TIM3->OR = (TIM3->OR &~TIM3_OR_TI1_RMP) | (((3) << 0) & TIM3_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM3->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM3CH2toPIN)
	TIM3CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr3IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic3
void tmr3IC3Init(uint8_t ps) {
	//set up the capture channel 1
	TIM3->CCMR2 = (TIM3->CCMR2 &~TIM_CCMR2_CC3S) | (TIM_CCMR2_CC3S_0);	//0b01->input capture
	TIM3->CCMR2 = (TIM3->CCMR2 &~TIM_CCMR2_IC3PSC) | ((ps<<TIM_CCMR2_IC3PSC_Pos) & TIM_CCMR2_IC3PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM3->SR &=~TIM_SR_CC3IF; TIM3->DIER &=~TIM_DIER_CC3IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM3->CCER |= TIM_CCER_CC3E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM3 rmp to MCO (=3)
	//TIM3->OR = (TIM3->OR &~TIM3_OR_TI1_RMP) | (((3) << 0) & TIM3_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM3->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM3CH3toPIN)
	TIM3CH3toPIN();											//configure the input capture pin
#endif	//ch3
}

//activate the isr handler
void tmr3IC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic4
void tmr3IC4Init(uint8_t ps) {
	//set up the capture channel 1
	TIM3->CCMR2 = (TIM3->CCMR2 &~TIM_CCMR2_CC4S) | (TIM_CCMR2_CC4S_0);	//0b01->input capture
	TIM3->CCMR2 = (TIM3->CCMR2 &~TIM_CCMR2_IC4PSC) | ((ps<<TIM_CCMR2_IC4PSC_Pos) & TIM_CCMR2_IC4PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM3->SR &=~TIM_SR_CC4IF; TIM3->DIER &=~TIM_DIER_CC4IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM3->CCER |= TIM_CCER_CC4E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM3 rmp to MCO (=3)
	//TIM3->OR = (TIM3->OR &~TIM3_OR_TI1_RMP) | (((3) << 0) & TIM3_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM3->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM3CH4toPIN)
	TIM3CH4toPIN();											//configure the input capture pin
#endif	//ch4
}

//activate the isr handler
void tmr3IC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//tim3

#if defined(TIM6)		//TiM6 only available in x8 devices
//tim6
//global variables
static void (* _tim6_ovfisrptr)(void)=empty_handler;				//TIM6_ptr pointing to empty_handler by default
static void (* _tim6_cc1isrptr)(void)=empty_handler;				//TIM6_ptr pointing to empty_handler by default
static void (* _tim6_cc2isrptr)(void)=empty_handler;				//TIM6_ptr pointing to empty_handler by default

static uint16_t _tim6_cc1=0;				//output compare registers
static uint16_t _tim6_cc2=0;

//isr for timer1 capture / compare
void TIM6_IRQHandler(void) {
	//oc1 portion
	if (TIM6->SR & TIM_SR_CC1IF) {TIM6->SR &=~TIM_SR_CC1IF; if (TIM6->CCER & TIM_CCER_CC1E) _TIM6_cc1 = TIM6->CCR1; else TIM6->CCR1 += _tim6_cc1; _tim6_cc1isrptr();}
	if (TIM6->SR & TIM_SR_CC2IF) {TIM6->SR &=~TIM_SR_CC2IF; if (TIM6->CCER & TIM_CCER_CC2E) _TIM6_cc2 = TIM6->CCR2; else TIM6->CCR2 += _tim6_cc2; _tim6_cc2isrptr();}
	//if (TIM6->SR & TIM_SR_CC3IF) {TIM6->SR &=~TIM_SR_CC3IF; if (TIM6->CCER & TIM_CCER_CC3E) _TIM6_cc3 = TIM6->CCR3; else TIM6->CCR3 += _tim6_cc3; _tim6_cc3isrptr();}
	//if (TIM6->SR & TIM_SR_CC4IF) {TIM6->SR &=~TIM_SR_CC4IF; if (TIM6->CCER & TIM_CCER_CC4E) _TIM6_cc4 = TIM6->CCR4; else TIM6->CCR4 += _tim6_cc4; _tim6_cc4isrptr();}

	//overflow isr
	if (TIM6->SR & TIM_SR_UIF) {TIM6->SR &=~TIM_SR_UIF; _tim3_ovfisrptr();}
}

//initialize the timer1 (16bit)
void tmr6Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	//reset timer
	tmrInit(TIM6, ps);
}

//activate the isr handler
void tmr6OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM6_IRQn);			//disable irq

	_tim6_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM6->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM6->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM6_IRQn);			//enable irq
	//priorities not set -> default values used.
}
#endif	//tim6

#if defined(TIM14)
//tim14
//global variables
static void (* _tim14_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim14_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim14_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
//static void (* _tim14_cc3isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
//static void (* _tim14_cc4isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static uint16_t _tim14_cc1=0;				//output compare registers
static uint16_t _tim14_cc2=0;
//static uint16_t _tim14_cc3=0;
//static uint16_t _tim14_cc4=0;

//isr for timer14
void TIM14_IRQHandler(void) {
	//capture / compare 1portion
	if ((TIM14->SR & TIM_SR_CC1IF) && (TIM14->DIER & TIM_DIER_CC1IE)) {TIM14->SR &=~TIM_SR_CC1IF; if (TIM14->CCER & TIM_CCER_CC1E) _tim14_cc1 = TIM14->CCR1; else TIM14->CCR1 += _tim14_cc1; _tim14_cc1isrptr();}
	//if ((TIM14->SR & TIM_SR_CC2IF) && (TIM14->DIER & TIM_DIER_CC2IE)) {TIM14->SR &=~TIM_SR_CC2IF; if (TIM14->CCER & TIM_CCER_CC2E) _tim14_cc2 = TIM14->CCR2; else TIM14->CCR2 += _tim14_cc2; _tim14_cc2isrptr();}
	//if ((TIM14->SR & TIM_SR_CC3IF) && (TIM14->DIER & TIM_DIER_CC3IE)) {TIM14->SR &=~TIM_SR_CC3IF; if (TIM14->CCER & TIM_CCER_CC3E) _tim14_cc3 = TIM14->CCR3; else TIM14->CCR3 += _tim14_cc3; _tim14_cc3isrptr();}
	//if ((TIM14->SR & TIM_SR_CC4IF) && (TIM14->DIER & TIM_DIER_CC4IE)) {TIM14->SR &=~TIM_SR_CC4IF; if (TIM14->CCER & TIM_CCER_CC4E) _tim14_cc4 = TIM14->CCR4; else TIM14->CCR4 += _tim14_cc4; _tim14_cc4isrptr();}

	//overflow isr
	if (TIM14->SR & TIM_SR_UIF) {TIM14->SR &=~TIM_SR_UIF; _tim14_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr14Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

	//reset timer
	tmrInit(TIM14, ps);
}

//install user handler
void tmr14OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);		//disable irq

	_tim14_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM14_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr14OC1SetPR(uint16_t pr) {
	//save the period value
	_tim14_cc1 = pr - 0;
	TIM14->CCR1 = _tim14_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr14OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);			//disable irq

	_tim14_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr14IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM14->CCMR1 = (TIM14->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM14->CCMR1 = (TIM14->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM14->SR &=~TIM_SR_CC1IF; TIM14->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM14->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM14 rmp to MCO (=3)
	//TIM14->OR = (TIM14->OR &~TIM14_OR_TI1_RMP) | (((3) << 0) & TIM14_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM14->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM14CH1toPIN)
	TIM14CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr14IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);		//disable irq

	_tim14_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif		//tim14

#if defined(TIM15)
//tim15
//global variables
static void (* _tim15_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim15_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim15_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static uint16_t _tim15_cc1=0;
static uint16_t _tim15_cc2=0;

//isr for timer1 capture / compare
void TIM15_IRQHandler(void) {
	//cc1 isr
	if ((TIM15->SR & TIM_SR_CC1IF) && (TIM15->DIER & TIM_DIER_CC1IE)) {TIM15->SR &=~TIM_SR_CC1IF; if (TIM15->CCER & TIM_CCER_CC1E) _tim15_cc1 = TIM14->CCR1; else TIM15->CCR1 += _tim15_cc1; _tim15_cc1isrptr();}
	if ((TIM15->SR & TIM_SR_CC2IF) && (TIM15->DIER & TIM_DIER_CC2IE)) {TIM15->SR &=~TIM_SR_CC2IF; if (TIM15->CCER & TIM_CCER_CC2E) _tim15_cc1 = TIM14->CCR2; else TIM15->CCR2 += _tim15_cc2; _tim15_cc2isrptr();}
	
	//ovf isr
	if ((TIM15->SR & TIM_SR_UIF) && (TIM15->DIER & TIM_DIER_UIE)) {TIM15->SR &=~TIM_SR_UIF; _tim15_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr15Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	//reset timer
	tmrInit(TIM15, ps);
}

//install user handler
void tmr15OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set TIM15_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr15OC1SetPR(uint16_t pr) {
	//save the period value
	_tim15_cc1 = pr - 0;
	TIM15->CCR1 = _tim15_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr15OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);			//disable irq

	_tim15_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM15_cc2 period
//pr is 16-bit. 32-bit used for compatability;
void tmr15OC2SetPR(uint16_t pr) {
	//save the period value
	_tim15_cc2 = pr - 0;
	TIM15->CCR2 = _tim15_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr15OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);			//disable irq

	_tim15_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);			//enable irq
	//priorities not set -> default values used.
}


//ic1
void tmr15IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM15->CCMR1 = (TIM15->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM15->CCMR1 = (TIM15->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM15->SR &=~TIM_SR_CC1IF; TIM15->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM15->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM15 rmp to MCO (=3)
	//TIM15->OR = (TIM15->OR &~TIM15_OR_TI1_RMP) | (((3) << 0) & TIM15_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM15->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM15CH1toPIN)
	TIM15CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr15IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr15IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM15->CCMR1 = (TIM15->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM15->CCMR1 = (TIM15->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM15->SR &=~TIM_SR_CC2IF; TIM15->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM15->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM15 rmp to MCO (=3)
	//TIM15->OR = (TIM15->OR &~TIM15_OR_TI1_RMP) | (((3) << 0) & TIM15_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM15->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM15CH2toPIN)
	TIM15CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr15IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif	//tim15

#if defined(TIM16)
//tim16
//global variables
static void (* _tim16_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim16_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim16_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static uint16_t _tim16_cc1=0;
static uint16_t _tim16_cc2=0;

//isr for timer1 capture / compare
void TIM16_IRQHandler(void) {
	//cc1 isr
	if ((TIM16->SR & TIM_SR_CC1IF) && (TIM16->DIER & TIM_DIER_CC1IE)) {TIM16->SR &=~TIM_SR_CC1IF; if (TIM16->CCER & TIM_CCER_CC1E) _tim16_cc1 = TIM16->CCR1; else TIM16->CCR1 += _tim16_cc1; _tim16_cc1isrptr();}
	if ((TIM16->SR & TIM_SR_CC2IF) && (TIM16->DIER & TIM_DIER_CC2IE)) {TIM16->SR &=~TIM_SR_CC2IF; if (TIM16->CCER & TIM_CCER_CC2E) _tim16_cc1 = TIM16->CCR2; else TIM16->CCR2 += _tim16_cc2; _tim16_cc2isrptr();}
	
	//ovf isr
	if ((TIM16->SR & TIM_SR_UIF) && (TIM16->DIER & TIM_DIER_UIE)) {TIM16->SR &=~TIM_SR_UIF; _tim16_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr16Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM16EN;

	//reset timer
	tmrInit(TIM16, ps);
}

//install user handler
void tmr16OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set TIM16_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr16OC1SetPR(uint16_t pr) {
	//save the period value
	_tim16_cc1 = pr - 0;
	TIM16->CCR1 = _tim16_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr16OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);			//disable irq

	_tim16_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM16_cc2 period
//pr is 16-bit. 32-bit used for compatability;
void tmr16OC2SetPR(uint16_t pr) {
	//save the period value
	_tim16_cc2 = pr - 0;
	TIM16->CCR2 = _tim16_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr16OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);			//disable irq

	_tim16_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr16IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM16->CCMR1 = (TIM16->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM16->CCMR1 = (TIM16->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM16->SR &=~TIM_SR_CC1IF; TIM16->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM16->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM16 rmp to MCO (=3)
	//TIM16->OR = (TIM16->OR &~TIM16_OR_TI1_RMP) | (((3) << 0) & TIM16_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM16->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM16CH1toPIN)
	TIM16CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr16IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr16IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM16->CCMR1 = (TIM16->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM16->CCMR1 = (TIM16->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM16->SR &=~TIM_SR_CC2IF; TIM16->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM16->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM16 rmp to MCO (=3)
	//TIM16->OR = (TIM16->OR &~TIM16_OR_TI1_RMP) | (((3) << 0) & TIM16_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM16->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM16CH2toPIN)
	TIM16CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr16IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//tim16

#if defined(TIM17)
//tim17
//global variables
static void (* _tim17_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim17_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim17_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static uint16_t _tim17_cc1=0;
static uint16_t _tim17_cc2=0;

//isr for timer1 capture / compare
void TIM17_IRQHandler(void) {
	//cc1 isr
	if ((TIM17->SR & TIM_SR_CC1IF) && (TIM17->DIER & TIM_DIER_CC1IE)) {TIM17->SR &=~TIM_SR_CC1IF; if (TIM17->CCER & TIM_CCER_CC1E) _tim17_cc1 = TIM17->CCR1; else TIM17->CCR1 += _tim17_cc1; _tim17_cc1isrptr();}
	if ((TIM17->SR & TIM_SR_CC2IF) && (TIM17->DIER & TIM_DIER_CC2IE)) {TIM17->SR &=~TIM_SR_CC2IF; if (TIM17->CCER & TIM_CCER_CC2E) _tim17_cc1 = TIM17->CCR2; else TIM17->CCR2 += _tim17_cc2; _tim17_cc2isrptr();}
	
	//ovf isr
	if ((TIM17->SR & TIM_SR_UIF) && (TIM17->DIER & TIM_DIER_UIE)) {TIM17->SR &=~TIM_SR_UIF; _tim17_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//17-bit prescaler. 32-bit used for compatability
void tmr17Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM17EN;

	//reset timer
	tmrInit(TIM17, ps);
}

//install user handler
void tmr17OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set TIM17_cc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr17OC1SetPR(uint16_t pr) {
	//save the period value
	_tim17_cc1 = pr - 0;
	TIM17->CCR1 = _tim17_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr17OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);			//disable irq

	_tim17_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM17_cc2 period
//pr is 16-bit. 32-bit used for compatability;
void tmr17OC2SetPR(uint16_t pr) {
	//save the period value
	_tim17_cc2 = pr - 0;
	TIM17->CCR2 = _tim17_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr17OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);			//disable irq

	_tim17_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr17IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM17->CCMR1 = (TIM17->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM17->CCMR1 = (TIM17->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM17->SR &=~TIM_SR_CC1IF; TIM17->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM17->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM17 rmp to MCO (=3)
	//TIM17->OR = (TIM17->OR &~TIM17_OR_TI1_RMP) | (((3) << 0) & TIM17_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM17->EGR |=TIM_EGR_UG;								//force an update


#if defined(TIM17CH1toPIN)
	TIM17CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr17IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr17IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM17->CCMR1 = (TIM17->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM17->CCMR1 = (TIM17->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM17->SR &=~TIM_SR_CC2IF; TIM17->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM17->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM17 rmp to MCO (=3)
	//TIM17->OR = (TIM17->OR &~TIM17_OR_TI1_RMP) | (((3) << 0) & TIM17_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM17->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM17CH2toPIN)
	TIM17CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr17IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif		//tim17

//chained timers

#if defined(TIM3) && defined(TIM1)
//initialize tim31 as a synchronized 32-bit timer
//tim3 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr31Init(uint16_t ps) {
	//initialize tim3 as master
	//enable clock to tim3
	tmr3Init(ps); TIM3->CR1 &=~TIM_CR1_CEN;	//configure tim3 but stop it
	TIM3->CR2 = 0x20;					//MMS = 0b010->tim3 as prescaler
	//initialize tim1 as slave
	tmr1Init(1); TIM1->CR1 &=~TIM_CR1_CEN;	//configure tim2 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM1->SMCR = (TIM1->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x02 << 4) | 				//tab 82: 0b010->//slave tim1 driven by tim3
			(0x07 << 0) |				//0b111->external trigger on trgo
			TIM_SMCR_MSM |					//1->enable slave mode
			0x00;

	TIM1->CNT = TIM3->CNT = 0;			//reset the conters
	
	//enable the timer, msw first
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//read 32-bit timer value
uint32_t tmr31Get(void) {
	uint16_t msw, lsw;					//timer's high/low words

	//double read to maintain atomicity
	do {
		msw = TIM1->CNT;				//read the msw
		lsw = TIM3->CNT;				//read the lsw
	} while (msw != TIM1->CNT);			//see if overflow has taken place
	return ((uint32_t) msw << 16) | lsw;			//return 32-bit time
}

//set 32-bit timer period
uint32_t tmr31SetPR(uint32_t pr) {
	TIM3->ARR = (pr - 1);
	TIM1->ARR = (pr - 1)>> 16;
	return pr;
}

//read 32-bit timer period
uint32_t tmr31GetPR(void) {
	uint32_t tmp;
	
	tmp = TIM1->ARR;
	tmp = ((tmp << 16) | TIM3->ARR) + 1;
	return tmp;
}
#endif	//tim31


#if defined(TIM17) && defined(TIM1)
//initialize tim171 as a synchronized 32-bit timer
//tim17 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr171Init(uint16_t ps) {
	//initialize tim17 as master
	//enable clock to tim17
	tmr17Init(ps); TIM17->CR1 &=~TIM_CR1_CEN;	//configure tim2 but stop it
	TIM17->CR2 = 0x40;					//MMS = 0b100->tim16 oc1ref as prescaler 
	TIM17->CCER&=~TIM_CCER_CC1E;		//disable cc1
	TIM17->CCMR1 = (TIM17->CCMR1 &~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S)) | (6<<4) | (0<<0);	//6 or 7works, 0->as output
	TIM17->ARR =-1;
	TIM17->CCR1=TIM17->ARR;
	TIM17->BDTR|= TIM_BDTR_MOE;			//1->set MOE - must have. I think this is a bug
	//TIM16->EGR|= TIM_EGR_CC1G;			//enable cc1g event generation
	//TIM16->SMCR=(TIM16->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) | 
	//			(0x02 << 4) |			//2->table 74: tim16 drives tim15
	//			(0x07<<0);				//0->channel as output
	TIM17->CCER|=TIM_CCER_CC1E;			//enable cc1

	//initialize tim1 as slave
	tmr1Init(1); TIM1->CR1 &=~TIM_CR1_CEN;	//configure tim1 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM1->SMCR = (TIM1->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x03 << 4) | 				//tab 39: 0b011->//slave TIM1 driven by TIM17 oc1ref
			(0x07 << 0) |				//0b110->trigger mode, 0b111->external trigger on trgo
			TIM_SMCR_MSM |				//1->slave mode
			0x00;

	TIM1->CNT = TIM17->CNT = 0;			//reset the conters
	
	//enable the timer, msw first
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//read 32-bit timer value
uint32_t tmr171Get(void) {
	uint16_t msw, lsw;					//timer's high/low words

	//double read to maintain atomicity
	do {
		msw = TIM1->CNT;				//read the msw
		lsw = TIM17->CNT;				//read the lsw
	} while (msw != TIM1->CNT);			//see if overflow has taken place
	return ((uint32_t) msw << 16) | lsw;			//return 32-bit time
}

//set 32-bit timer period
uint32_t tmr171SetPR(uint32_t pr) {
	TIM17->ARR = TIM17->CCR1 = (pr - 1);
	TIM1->ARR = (pr - 1)>> 16;
	return pr;
}

//read 32-bit timer period
uint32_t tmr171GetPR(void) {
	uint32_t tmp;
	
	tmp = TIM1->ARR;
	tmp = ((tmp << 16) | TIM17->ARR) + 1;
	return tmp;
}
#endif	//tim31

#if defined(TIM1) && defined(TIM3)
//initialize tim13 as a synchronized 32-bit timer
//tim1 as master / lsw (including prescaler) to tim3 / msw of the 32-bit timer
//tim3 as slave / msw of the 32-bit timer
void tmr13Init(uint16_t ps) {
	//initialize tim1 as master
	//enable clock to tim1
	tmr1Init(ps); TIM1->CR1 &=~TIM_CR1_CEN;	//configure tim3 but stop it
	TIM1->CR2 = 0x20;					//MMS = 0b010->tim3 as prescaler
	//initialize tim3 as slave
	tmr3Init(1); TIM3->CR1 &=~TIM_CR1_CEN;	//configure tim2 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM3->SMCR = (TIM3->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x00 << 4) | 				//tab 43: 0b000->//slave tim3 driven by tim1
			(0x07 << 0) |				//0b111->external trigger on trgo
			TIM_SMCR_MSM |					//1->enable slave mode
			0x00;
	
	TIM3->CNT = TIM1->CNT = 0;			//reset the conters
	
	//enable the timer, msw first
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//read 32-bit timer value
uint32_t tmr13Get(void) {
	uint16_t msw, lsw;					//timer's high/low words

	//double read to maintain atomicity
	do {
		msw = TIM3->CNT;				//read the msw
		lsw = TIM1->CNT;				//read the lsw
	} while (msw != TIM3->CNT);			//see if overflow has taken place
	return ((uint32_t) msw << 16) | lsw;			//return 32-bit time
}

//set 32-bit timer period
uint32_t tmr13SetPR(uint32_t pr) {
	TIM1->ARR = (pr - 1);
	TIM3->ARR = (pr - 1)>> 16;
	return pr;
}

//read 32-bit timer period
uint32_t tmr13GetPR(void) {
	uint32_t tmp;
	
	tmp = TIM3->ARR;
	tmp = ((tmp << 16) | TIM1->ARR) + 1;
	return tmp;
}
#endif	//tim13

#if defined(TIM14) && defined(TIM3)
//initialize tim143 as a synchronized 32-bit timer
//tim14 as master / lsw (including prescaler) to tim3 / msw of the 32-bit timer
//tim3 as slave / msw of the 32-bit timer
void tmr143Init(uint16_t ps) {
	//initialize tim17 as master
	//enable clock to tim17
	tmr14Init(ps); TIM14->CR1 &=~TIM_CR1_CEN;	//configure tim2 but stop it
	TIM14->CR2 = 0x40;					//MMS = 0b100->tim16 oc1ref as prescaler 
	TIM14->CCER&=~TIM_CCER_CC1E;		//disable cc1
	TIM14->CCMR1 = (TIM14->CCMR1 &~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S)) | (6<<4) | (0<<0);	//6 or 7works, 0->as output
	TIM14->ARR =-1;
	TIM14->CCR1=TIM14->ARR;
	TIM14->BDTR|= TIM_BDTR_MOE;			//1->set MOE - must have. I think this is a bug
	//TIM16->EGR|= TIM_EGR_CC1G;			//enable cc1g event generation
	//TIM16->SMCR=(TIM16->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) | 
	//			(0x02 << 4) |			//2->table 74: tim16 drives tim15
	//			(0x07<<0);				//0->channel as output
	TIM14->CCER|=TIM_CCER_CC1E;			//enable cc1

	//initialize tim1 as slave
	tmr3Init(1); TIM3->CR1 &=~TIM_CR1_CEN;	//configure tim1 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM3->SMCR = (TIM3->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x03 << 4) | 				//tab 43: 0b011->//slave TIM1 driven by TIM14 oc1ref
			(0x07 << 0) |				//0b110->trigger mode, 0b111->external trigger on trgo
			TIM_SMCR_MSM |				//1->slave mode
			0x00;

	TIM3->CNT = TIM14->CNT = 0;			//reset the conters
	
	//enable the timer, msw first
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer
	TIM14->CR1|= TIM_CR1_CEN;			//enable the timer
}

//read 32-bit timer value
uint32_t tmr143Get(void) {
	uint16_t msw, lsw;					//timer's high/low words

	//double read to maintain atomicity
	do {
		msw = TIM3->CNT;				//read the msw
		lsw = TIM14->CNT;				//read the lsw
	} while (msw != TIM3->CNT);			//see if overflow has taken place
	return ((uint32_t) msw << 16) | lsw;			//return 32-bit time
}

//set 32-bit timer period
uint32_t tmr143SetPR(uint32_t pr) {
	TIM14->ARR = TIM14->CCR1 = (pr - 1);
	TIM3->ARR = (pr - 1)>> 16;
	return pr;
}

//read 32-bit timer period
uint32_t tmr143GetPR(void) {
	uint32_t tmp;
	
	tmp = TIM3->ARR;
	tmp = ((tmp << 16) | TIM14->ARR) + 1;
	return tmp;
}
#endif	//tim143
//end Timer

//generic pwm set-up
void pwmInit(TIM_TypeDef *TIMx, uint16_t TxCCP_PS) {
	//source from internal clock -> disable slave mode
	TIMx->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM1->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM1->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM1->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM1->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM1->CR1 = 0;						//much easier
	TIMx->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIMx->CR2 = 0;						//default value
	TIMx->SMCR= 0;						//default value

	//set the prescaler
	TIMx->PSC = TxCCP_PS - 1;			//set the prescaler
	TIMx->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIMx->ARR = PWM_PR;					//auto reload register / period = 0; - need to change for downcounters
	TIMx->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIMx->SR &=~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIMx->DIER &=~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	TIMx->EGR = 0xff;					//force an update
	TIMx->BDTR|= TIM_BDTR_MOE;			//enable the main output
	//enable the timer.
	//TIMx->CR1 |= TIM_CR1_CEN;			//enable the timer
}

/*
MAPR Bits 7:6 TIM1_REMAP[1:0]: TIM1 remapping
These bits are set and cleared by software. They control the mapping of TIM1 channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN) on the GPIO ports.
00: No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
01: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
10: not used
11: Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm1Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

	pwmInit(TIM1, TxCCP_PS);			//initialize pwm
	
	//configure CCP1..4
#if defined(TIM1CH1toPIN) || defined(TIM1CH1NtoPIN)
	//configure CCP1
	TIM1->CCMR1 = 	(TIM1->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM1CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//tim1ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM1->CCR1 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH1toPIN)
	TIM1CH1toPIN();						//route TIM1CH1 to GPIO
#endif	//ch1
#if defined(TIM1CH1NtoPIN)
	TIM1CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

#if defined(TIM1CH2toPIN) || defined(TIM1CH2NtoPIN)
	//configure CCP2
	TIM1->CCMR1 = 	(TIM1->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
#if defined(TIM1CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif	//ch2n
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM1->CCR2 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH2toPIN)
	TIM1CH2toPIN();						//route TIM1CH1 to GPIO
#endif	//ch2
#if defined(TIM1CH2NtoPIN)
	TIM1CH2NtoPIN();
#endif	//ch2n
#endif	//ch2 || ch2n

#if defined(TIM1CH3toPIN) || defined(TIM1CH3NtoPIN)
	//configure CCP3
	TIM1->CCMR2 = 	(TIM1->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 8)) |
					(0<<11) |			//0->normal polarity for CC3N, 1->inverse polarity
#if defined(TIM1CH3NtoPIN)
					(1<<10) |			//0->disable CC3N, 1->enable CC3N
#else
					(0<<10) |			//0->disable CC3N, 1->enable CC3N
#endif	//ch3n
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM1->CCR3 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH3toPIN)
	TIM1CH3toPIN();					//route TIM1CH1 to GPIO
#endif	//ch3
#if defined(TIM1CH3NtoPIN)
	TIM1CH3NtoPIN();
#endif	//ch3n
#endif	//ch3 || ch3n

#if defined(TIM1CH4toPIN) || defined(TIM1CH4NtoPIN)
	//configure CCP4
	TIM1->CCMR2 = 	(TIM1->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
#if defined(TIM1CH4NtoPIN)
					(1<<14) |			//0->disable CC4N, 1->enable CC4N
#else
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
#endif	//ch4n
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM1->CCR4 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH4toPIN)
	TIM1CH4toPIN();						//route TIM1CH1 to GPIO
#endif	//ch4
#if defined(TIM1CH4NtoPIN)
	TIM1CH4NtoPIN();
#endif	//ch4n
#endif	//ch4 || ch4n

	//enable the timer.
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer

}

#if defined(TIM3)
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm3Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM3EN;

	pwmInit(TIM3, TxCCP_PS);			//initialize the pwm
	
	//configure CCP1..4
#if defined(TIM3CH1toPIN) || defined(TIM3CH1NtoPIN)
	//configure CCP1
	TIM3->CCMR1 = 	(TIM3->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM3CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM3->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH1toPIN)
	TIM3CH1toPIN();					//route TIM1CH1 to GPIO
#endif	//ch1
#if defined(TIM3CH1NtoPIN)
	TIM3CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

#if defined(TIM3CH2toPIN) || defined(TIM3CH2NtoPIN)
	//configure CCP2
	TIM3->CCMR1 = 	(TIM3->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
#if defined(TIM3CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif	//ch2n
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM3->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH2toPIN)
	TIM3CH2toPIN();					//route TIM1CH1 to GPIO
#endif	//ch2
#if defined(TIM3CH2NtoPIN)
	TIM3CH2NtoPIN();
#endif	//ch2n
#endif	//ch2 || ch2n

#if defined(TIM3CH3toPIN) || defined(TIM3CH3NtoPIN)
	//configure CCP3
	TIM3->CCMR2 = 	(TIM3->CCMR2 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 8)) |
					(0<<11) |			//0->normal polarity for CC3N, 1->inverse polarity
#if defined(TIM3CH3NtoPIN)
					(1<<10) |			//0->disable CC3N, 1->enable CC3N
#else
					(0<<10) |			//0->disable CC3N, 1->enable CC3N
#endif	//ch3n
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM3->CCR3 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH3toPIN)
	TIM3CH3toPIN();					//route TIM1CH1 to GPIO
#endif	//ch3
#if defined(TIM3CH3NtoPIN)
	TIM3CH3NtoPIN();
#endif	//ch3n
#endif	//ch3 || ch3n

#if defined(TIM3CH4toPIN) || defined(TIM3CH4NtoPIN)
	//configure CCP4
	TIM3->CCMR2 = 	(TIM3->CCMR2 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
#if defined(TIM3CH4NtoPIN)
					(1<<14) |			//0->disable CC4N, 1->enable CC4N
#else
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
#endif	//ch4n
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM3->CCR4 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH4toPIN)
	TIM3CH4toPIN();					//route TIM1CH1 to GPIO
#endif	//ch4
#if defined(TIM3CH4NtoPIN)
	TIM3CH4NtoPIN();
#endif	//ch4n
#endif	//ch4 || ch4n

	//TIM3->EGR = 0xff;					//force an update
	//TIM3->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//tim3

#if defined(TIM14)
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm14Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

	pwmInit(TIM14, TxCCP_PS);			//initialize the pwm
	
	//configure CCP1..4
#if defined(TIM14CH1toPIN) || defined(TIM14CH1NtoPIN)
	//configure CCP1
	TIM14->CCMR1 = 	(TIM14->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM14->CCER = 	(TIM14->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM14CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM14->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM14CH1toPIN)
	TIM14CH1toPIN();					//route TIM1CH1 to GPIO
#endif	//ch1
#if defined(TIM14CH1NtoPIN)
	TIM14CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

	//TIM14->EGR = 0xff;					//force an update
	//TIM14->BDTR|= TIM_BDTR_MOE;			//enable the main output
	//enable the timer.
	TIM14->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//tim14

#if defined(TIM15)
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	pwmInit(TIM15CH1NtoPB15, TxCCP_PS);		//initialize the pwm

	//configure CCP1..4
#if defined(TIM15CH1toPIN) || defined(TIM15CH1NtoPIN)
	//configure CCP1
	TIM15->CCMR1 = 	(TIM15->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM15->CCER = 	(TIM15->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM15CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM15->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM15CH1toPIN)
	TIM15CH1toPIN();					//route TIM1CH1 to GPIO
#endif	//ch1
#if defined(TIM15CH1NtoPIN)
	TIM15CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

#if defined(TIM15CH2toPIN) || defined(TIM14CH2NtoPIN)
	//configure CCP2
	TIM15->CCMR1 = 	(TIM15->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM15->CCER = 	(TIM15->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
#if defined(TIM15CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif	//ch2n
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM15->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM15CH2toPIN)
	TIM15CH2toPIN();					//route TIM1CH1 to GPIO
#endif	//ch2
#if defined(TIM15CH2NtoPIN)
	TIM15CH2NtoPIN();
#endif	//ch2n
#endif	//ch2 || ch2n

	TIM15->EGR = 0xff;					//force an update
	TIM15->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM15->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//tim15

/*
MAPR2 Bit 1 TIM16_REMAP: TIM16 remapping
This bit is set and cleared by software. It controls the mapping of the alternate functions of TIM16 channel 1 onto the GPIO ports.
0: No remap (CH1/PB8)
1: Remap (CH1/PA6)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm16Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM16EN;

	pwmInit(TIM16, TxCCP_PS);			//initialize the pwm
	
	//configure CCP1..4
#if defined(TIM16CH1toPIN) || defined(TIM16CH1NtoPIN)
	//configure CCP1
	TIM16->CCMR1 = 	(TIM16->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM16->CCER = 	(TIM16->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM16CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM16->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM16CH1toPIN)
	TIM16CH1toPIN();					//route TIM1CH1 to GPIO
#endif	//ch1
#if defined(TIM16CH1NtoPIN)
	TIM16CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

	//TIM16->EGR = 0xff;					//force an update
	//TIM16->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM16->CR1 |= TIM_CR1_CEN;			//enable the timer

}

#if defined(TIM17)
/*
MAPR2 Bit 2 TIM17_REMAP: TIM17 remapping
This bit is set and cleared by software. It controls the mapping of the alternate functions of TIM17 channel 1 onto the GPIO ports.
0: No remap (CH1/PB9)
1: Remap (CH1/PA7)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm17Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM17EN;

	pwmInit(TIM17, TxCCP_PS);			//initialize the pwm
	
	//configure CCP1..4
#if defined(TIM17CH1toPIN) || defined(TIM17CH1NtoPIN)
	//configure CCP1
	TIM17->CCMR1 = 	(TIM17->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM17->CCER = 	(TIM17->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM17CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif	//ch1n
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM17->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM17CH1toPIN)
	TIM17CH1toPIN();					//route TIM1CH1 to GPIO
#endif	//ch1n
#if defined(TIM17CH1NtoPIN)
	TIM17CH1NtoPIN();
#endif	//ch1n
#endif	//ch1 || ch1n

	//TIM17->EGR = 0xff;					//force an update
	//TIM17->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//tim17

//lptimer
//genertic set up routine
void lptmrInit(LPTIM_TypeDef *LPTIMx, uint16_t ps) {
	//set up lptimer
	LPTIMx->CR &=~LPTIM_CR_ENABLE;				//1->enable the lptimer, 0->disable the lptimer
	LPTIMx->CFGR = 	((ps << LPTIM_CFGR_PRESC_Pos) & LPTIM_CFGR_PRESC) |	//set prescaler
					(0<<22) |					//0->no preload, 1->enable preload
					0x00;
	LPTIMx->CR = 	(0<<4) |					//0->reading lptim counter doesn't reset it, 1->reading lptim counter resets it
					(0<<1) |					//LPTIM_CR_SNGSTRT |				//ENABLE single shot mode -> the only mode available
					0x00;
	LPTIMx->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	LPTIMx->IER&=~LPTIM_IER_ARRMIE;				//0->disable the interrupt
	LPTIMx->CR |= LPTIM_CR_ENABLE;				//1->enable the lptimer, 0->disable the lptimer
	LPTIMx->ARR = -1;							//ARR at the top - can write to only after lptim has been enabled
}

//global variables
static void (* _lptim1_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static uint16_t _lptim1_pr=0;

void LPTIM1_IRQHandler(void) {
	LPTIM1->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	//may want to process this in the user isr _lptim1_isrptr()
	LPTIM1->CR |= LPTIM_CR_SNGSTRT;				//1->start the timer, 0->stop the timer
	_lptim1_isrptr();							//execute user isr
}

//initialize lptimer
void lptmr1Init(uint8_t ps) {
	RCC->APBENR1 |= RCC_APBENR1_LPTIMEN;	//enable clock to lptimer
	
	lptmrInit(LPTIM1, ps);				//setup the lptimer
}

//install user handler
void lptmr1AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(LPTIM1_IRQn);		//disable irq

	_lptim1_isrptr = isr_ptr;			//install user handler

	//clear the flag
	LPTIM1->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	LPTIM1->IER|= LPTIM_IER_ARRMIE;				//0->disable the interrupt

	NVIC_EnableIRQ(LPTIM1_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//end lptimer

//adc module
//global variables
static uint32_t _adc_calfactor;				//adc calibration factor
//initialize adc: 12-bit, single conversion mode
void adc1Init(void) {
	uint32_t tmp;
	
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;		//enable clock to ADC1

	//configure adc
	ADC1->CR&=~ADC_CR_ADEN;				//1->enable adc, 0->disable adc: can only configure if adc is enabled
	//ADC1->CR|= ADC_CR_ADSTP;				//1->stop the adc, 0->no effect
	//while (ADC1->CR & ADC_CR_ADSTP) continue;	//wait for the current adc conversion to stop
	ADC1->CR|= ADC_CR_ADEN;				//1->enable adc, 0->disable adc: can only configure if adc is enabled
	ADC1->CFGR1 = 	(0<<26) |				//0->awd on channel 0
					(0<<23) |				//0->AWD disabled
					(0<<22) |				//0->AWD on all channels (but disabled by bit 22
					(0<<16) |				//1->enable discontinuous conversion
					(1<<15) |				//0->auto off disabled
					(0<<14) |				//0->wait conversion disabled
					(0<<13) |				//0->single conversion mode, 1->continuous mode
					(0<<12) |				//0->adc data register preserved in overrun
					(0<<10) |				//00->hardware external trigger disabled
					(0<<6) |				//0000->external on TRG0 - but disabled
					(0<<5) |				//0->right aligned, 1->left aligned
					(0<<3) |				//00->data resolution = 12bit, 01->10bit, 10->8bit, 11->6bit
					(0<<2) |				//0->upward scan
					(0<<1) |				//0->DMA one shot mode selected
					(0<<0) |				//0->DMA disabled
					0x00;
	ADC1->CFGR2 = 	(4 << ADC_CFGR2_CKMODE_Pos);			//00->adc clock, 01->PCLK/2, 10->PCLK/4 -> no jitter
	//set adc sample time
	//0b111->239.5 cycles for all channels
	//set up smp time based on f_pclk
	if (F_PCLK > 4000000ul) tmp = 7; else tmp = 6;			//0->3.5 adc clock, 1->5.5, 2->7.5, 3->13.5, 4->28.5, 5->41.5, 6->71.5, 7->239.5
	ADC1->SMPR = 	((tmp << ADC_SMPR_SMP_Pos) & ADC_SMPR_SMP) |	
					0x00;
	//set adc channel sequence
	//ADC1->SQR3 = ADC1->SQR2 = ADC1->SQR1 = 0;							//0->1 conversion
	
	//start self-calibration - system calibrate the adc automatically on power-on
#if 0
	ADC1->CR|= ADC_CR_ADEN;					//1->enable adc, 0->disable adc
	ADC1->CCSR = (ADC1->CCSR &~ADC_CCSR_CALSMP) | (2 << ADC_CCSR_CALSMP_Pos);	//2->adc calibration smp: 0->2 adc clock, 1->4 adc clock, 2->8 adc clock, 3->1 adc clock
	ADC1->CCSR = (ADC1->CCSR &~ADC_CCSR_CALSEL) | (0 << ADC_CCSR_CALSEL_Pos);	//calibration context: 0->offset only, 1->offst and linearity
	ADC1->CR|= ADC_CR_ADCAL;				//1->start the calibration
	while (ADC1->CR & ADC_CR_ADCAL) continue;	//wait for ADC calibration to finish
	while (ADC1->CCSR & (ADC_CCSR_CALON | ADC_CCSR_CALFAIL)) continue;	//hang here if adc calibration error (calon and calfail <> 0)
	_adc_calfactor = ADC1->DR;				//save adc calibration factor - offset
#endif	//0

	//optional: enable temperature sensors
	ADC->CCR |= 	ADC_CCR_VREFEN |		//1->enable temperature sensor
					ADC_CCR_TSEN |			//1->enable Vrefint. 1.20v nominal
					0x00;

	//reset interrupt flags / interrupts
	ADC1->ISR = 0;							//CLEAR_BIT the flag
	ADC1->IER = 0;							//0->disable interrupt

}

//analog to digital converter on ADC1
//ain/analog input channel: ain valid values: 0..15, 16=temperature sensor, 17=Vrefint
//***user needs to be configured as floating input***
int16_t adc1Read(uint32_t ain) {
	//ADC1->ISR &= ~(1<<2);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);					//start the conversion
	//adc is turned off at the end of each conversion
	ADC1->CFGR1 = (ADC1->CFGR1 &~ADC_CFGR1_AWDCH) | ((ain << ADC_CFGR1_AWDCH_Pos) & ADC_CFGR1_AWDCH);
	ADC1->CHSELR = (1ul<<ain) & 0x1ffful;	//define the first (and only) adc ch
	ADC1->CR |= ADC_CR_ADEN;				//1->enable the adc, 0->disable the adc
	ADC1->CR |= ADC_CR_ADSTART;				//start conversion
	while (ADC1->CR & ADC_CR_ADSTART) continue;	//wait for conversion to end (EOC set at end of conversion)
	//while ((ADC1->ISR & ADC_ISR_EOSEQ) == 0) continue;
	ADC1->ISR |= ADC_ISR_EOSEQ;				//write 1 to clear the flag
	return ADC1->DR;						//return adc result and clears the EOC bit
}
//end ADC

#if 0
//input capture
static void (*_ic1_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC1DAT=0;						//buffer 

//input capture ISR
void _ISR_PSV _IC1Interrupt(void) {				//for PIC32
	//clear the flag
	IC1DAT = IC1BUF;							//read the captured value
	IFS0bits.IC1IF = 0;							//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic1_isrptr();								//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1Init(void) {
	_ic1_isrptr = empty_handler;				//reset user handler

	IC12RP();									//assign pin to IC
	PMD2bits.IC1MD = 0;							//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC1CON1 = 0;						//reset to default value
	IC1CON2 = 0;						//reset to default value
	IC1CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC1CON1bits.ON = 1;					//1->enable the module, 0->disable the module
#else
	//for ga002 devices
	IC1CON = 0;						//reset to default value
	//IC1CON2 = 0;						//reset to default value
	IC1CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC1CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
	//input capture running now
}

//activate user ptr
void ic1AttachISR(void (*isrptr)(void)) {
	_ic1_isrptr = isrptr;				//install user ptr
	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic1Get(void) {
	return IC1DAT;
}
	
static void (*_ic2_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC2DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC2Interrupt(void) {		//for PIC32
	//clear the flag
	IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC2IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic2_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic2Init(void) {
	_ic2_isrptr = empty_handler;		//reset user handler

	IC22RP();							//assign pin to IC
	PMD2bits.IC2MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC2CON1 = 0;						//reset to default value
	IC2CON2 = 0;						//reset to default value
	IC2CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CON1bits.ON = 1;				//1->enable the module, 0->disable the module
#else
	//for ga002 devices
	IC2CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC2CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic2AttachISR(void (*isrptr)(void)) {
	_ic2_isrptr = isrptr;				//install user ptr
	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic2Get(void) {
	return IC2DAT;
}
	
static void (*_ic3_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC3DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC3Interrupt(void) {		//for PIC32
	//clear the flag
	IC3DAT = IC3BUF;					//read the captured value
	IFS2bits.IC3IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic3_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic3Init(void) {
	_ic3_isrptr = empty_handler;		//reset user handler

	IC32RP();							//assign pin to IC
	PMD2bits.IC3MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC3CON1 = 0;						//reset to default value
	IC3CON2 = 0;						//reset to default value
	IC3CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC3CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC3CON = 0;							//reset to default value
	//IC3CON2 = 0;						//reset to default value
	IC3CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC3CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic3AttachISR(void (*isrptr)(void)) {
	_ic3_isrptr = isrptr;				//install user ptr
	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic3Get(void) {
	return IC3DAT;
}
	
static void (*_ic4_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC4DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC4Interrupt(void) {		//for PIC32
	//clear the flag
	IC4DAT = IC4BUF;					//read the captured value
	IFS2bits.IC4IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic4_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic4Init(void) {
	_ic4_isrptr = empty_handler;		//reset user handler

	IC42RP();							//assign pin to IC
	PMD2bits.IC4MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC4CON1 = 0;						//reset to default value
	IC4CON2 = 0;						//reset to default value
	IC4CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC4CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC4CON = 0;						//reset to default value
	//IC4CON2 = 0;						//reset to default value
	IC4CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC4CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic4AttachISR(void (*isrptr)(void)) {
	_ic4_isrptr = isrptr;				//install user ptr
	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic4Get(void) {
	return IC4DAT;
}
	
static void (*_ic5_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC5DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC5Interrupt(void) {		//for PIC32
	//clear the flag
	IC5DAT = IC5BUF;					//read the captured value
	IFS2bits.IC5IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic5_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic5Init(void) {
	_ic5_isrptr = empty_handler;		//reset user handler

	IC52RP();							//assign pin to IC
	PMD2bits.IC5MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC5CON1 = 0;						//reset to default value
	IC5CON2 = 0;						//reset to default value
	IC5CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC5CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC5CON = 0;							//reset to default value
	//IC5CON2 = 0;						//reset to default value
	IC5CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC5CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic5AttachISR(void (*isrptr)(void)) {
	_ic5_isrptr = isrptr;				//install user ptr
	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic5Get(void) {
	return IC5DAT;
}
//end input capture
#endif

#if 0
//extint
//global variables
static void (* _exti0_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti1_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti2_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti3_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti4_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti5_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti6_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti7_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti8_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti9_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti10_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti11_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti12_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti13_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti14_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti15_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default

//EXTI0-1 ISR
void EXTI0_1_IRQHandler(void) {
	//clear the flag and execute user isr
	if (EXTI->PR & (1<< 0)) {EXTI->PR |= 1<< 0; _exti0_isrptr();}
	if (EXTI->PR & (1<< 1)) {EXTI->PR |= 1<< 1; _exti1_isrptr();}
}

//EXTI2-3 ISR
void EXTI2_3_IRQHandler(void) {
	//clear the flag and execute user isr
	if (EXTI->PR & (1<< 2)) {EXTI->PR |= 1<< 2; _exti2_isrptr();}
	if (EXTI->PR & (1<< 3)) {EXTI->PR |= 1<< 3; _exti3_isrptr();}
}

//EXTI15-10 ISR
void EXTI4_15_IRQHandler(void) {
	//clear the flag and execute user isr
	if (EXTI->PR & (1<< 4)) {EXTI->PR |= 1<< 4; _exti4_isrptr();}
	if (EXTI->PR & (1<< 5)) {EXTI->PR |= 1<< 5; _exti5_isrptr();}
	if (EXTI->PR & (1<< 6)) {EXTI->PR |= 1<< 6; _exti6_isrptr();}
	if (EXTI->PR & (1<< 7)) {EXTI->PR |= 1<< 7; _exti7_isrptr();}
	if (EXTI->PR & (1<< 8)) {EXTI->PR |= 1<< 8; _exti8_isrptr();}
	if (EXTI->PR & (1<< 9)) {EXTI->PR |= 1<< 9; _exti9_isrptr();}
	if (EXTI->PR & (1<<10)) {EXTI->PR |= 1<<10; _exti10_isrptr();}
	if (EXTI->PR & (1<<11)) {EXTI->PR |= 1<<11; _exti11_isrptr();}
	if (EXTI->PR & (1<<12)) {EXTI->PR |= 1<<12; _exti12_isrptr();}
	if (EXTI->PR & (1<<13)) {EXTI->PR |= 1<<13; _exti13_isrptr();}
	if (EXTI->PR & (1<<14)) {EXTI->PR |= 1<<14; _exti14_isrptr();}
	if (EXTI->PR & (1<<15)) {EXTI->PR |= 1<<15; _exti15_isrptr();}
}

//global defines
//global variables
//initialize the exti
//pin: one pin only
//gpio: GPIOA..GPIOG
void extiInit(PIN_TypeDef pinx, uint8_t edge) {
	uint16_t tmp=0;
	GPIO_TypeDef *gpio;
	uint16_t pin;
	
	gpio = GPIO_PinDef[pinx].gpio;
	pin  = GPIO_PinDef[pinx].mask;
	//enable afio
	//RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	//enable the clock
	switch ((uint32_t) gpio) {						//switch can only be of integer type
#if defined(GPIOA)
		case (uint32_t) GPIOA: RCC->IOPENR |= RCC_IOPENR_GPIOAEN; tmp = 0x00; break;
#endif
#if defined(GPIOB)
		case (uint32_t) GPIOB: RCC->IOPENR |= RCC_IOPENR_GPIOBEN; tmp = 0x01; break;
#endif
#if defined(GPIOC)
		case (uint32_t) GPIOC: RCC->IOPENR |= RCC_IOPENR_GPIOCEN; tmp = 0x02; break;
#endif
#if defined(GPIOD)
		case (uint32_t) GPIOD: RCC->IOPENR |= RCC_IOPENR_GPIODEN; tmp = 0x03; break;
#endif
#if defined(GPIOE)
		case (uint32_t) GPIOE: RCC->IOPENR |= RCC_IOPENR_GPIOEEN; tmp = 0x04; break;
#endif
#if defined(GPIOF)
		case (uint32_t) GPIOF: RCC->IOPENR |= RCC_IOPENR_GPIOFEN; tmp = 0x05; break;
#endif
#if defined(GPIOG)
		case (uint32_t) GPIOG: RCC->IOPENR |= RCC_IOPENR_GPIOGEN; tmp = 0x06; break;
#endif
	}
	//configure exti edges
	//1->falling/rising edge enabled, 0->falling/rising edge disabled
	switch (edge) {
		case EXTI_FALLING: 	EXTI->FTSR |= pin; EXTI->RTSR &=~pin; break;
		case EXTI_RISING: 	EXTI->FTSR &=~pin; EXTI->RTSR |= pin; break;
		case EXTI_BOTH: 	EXTI->FTSR |= pin; EXTI->RTSR |= pin; break;
	}
	//disable the interrupts
	EXTI->EXTICR[0] = EXTI->EXTICR[1]=EXTI->EXTICR[2]=/*EXTI->EXTICR[3]=*/0x00;
	//configure the port
}

//install user handler
void extiAttachISR(PIN_TypeDef pinx, void (*isr_ptr)(void)) {
	GPIO_TypeDef *gpio;
	uint16_t pin=0;
	uint16_t gpiox=0;
	
	gpio = GPIO_PinDef[pinx].gpio;
	pin  = GPIO_PinDef[pinx].mask;

	switch ((uint32_t) gpio) {
		case (uint32_t) GPIOA: gpiox=0; break;
#if defined(GPIOB)
		case (uint32_t) GPIOB: gpiox=1; break;
#endif
#if defined(GPIOC)
		case (uint32_t) GPIOC: gpiox=2; break;
#endif
#if defined(GPIOD)
		case (uint32_t) GPIOD: gpiox=3; break;
#endif
#if defined(GPIOE)
		case (uint32_t) GPIOE: gpiox=4; break;
#endif
#if defined(GPIOF)
		case (uint32_t) GPIOF: gpiox=5; break;
#endif
#if defined(GPIOG)
		case (uint32_t) GPIOG: gpiox=6; break;
#endif
	}
	
	pinMode(pinx, INPUT);						//default is floating input
	
	//clear the flag
	EXTI->PR |= pin;							//1->clear the flag
	//enable the interrupt
	EXTI->IMR |= pin;							//1->enable the interrupt, 0->disable the interrupt
	EXTI->EMR |= pin;							//1->enable the event trigger, 0->disable the event trigger

	//enable nvic
	switch (pin) {
		case (1<< 0): EXTI->EXTICR[0] |= gpiox << 0; NVIC_EnableIRQ(EXTI0_1_IRQn); _exti0_isrptr = isr_ptr; break;
		case (1<< 1): EXTI->EXTICR[0] |= gpiox << 8; NVIC_EnableIRQ(EXTI0_1_IRQn); _exti1_isrptr = isr_ptr; break;
		case (1<< 2): EXTI->EXTICR[0] |= gpiox <<16; NVIC_EnableIRQ(EXTI2_3_IRQn); _exti2_isrptr = isr_ptr; break;
		case (1<< 3): EXTI->EXTICR[0] |= gpiox <<24; NVIC_EnableIRQ(EXTI2_3_IRQn); _exti3_isrptr = isr_ptr; break;
		case (1<< 4): EXTI->EXTICR[1] |= gpiox << 0; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti4_isrptr = isr_ptr; break;
		case (1<< 5): EXTI->EXTICR[1] |= gpiox << 8; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti5_isrptr = isr_ptr; break;
		case (1<< 6): EXTI->EXTICR[1] |= gpiox <<16; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti6_isrptr = isr_ptr; break;
		case (1<< 7): EXTI->EXTICR[1] |= gpiox <<24; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti7_isrptr = isr_ptr; break;
		case (1<< 8): EXTI->EXTICR[2] |= gpiox << 0; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti8_isrptr = isr_ptr; break;
		case (1<< 9): EXTI->EXTICR[2] |= gpiox << 8; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti9_isrptr = isr_ptr; break;
		case (1<<10): EXTI->EXTICR[2] |= gpiox <<16; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti10_isrptr = isr_ptr; break;
		case (1<<11): EXTI->EXTICR[2] |= gpiox <<24; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti11_isrptr = isr_ptr; break;
		//case (1<<12): EXTI->EXTICR[3] |= gpiox << 0; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti12_isrptr = isr_ptr; break;
		//case (1<<13): EXTI->EXTICR[3] |= gpiox << 8; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti13_isrptr = isr_ptr; break;
		//case (1<<14): EXTI->EXTICR[3] |= gpiox <<16; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti14_isrptr = isr_ptr; break;
		//case (1<<15): EXTI->EXTICR[3] |= gpiox <<24; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti15_isrptr = isr_ptr; break;
	}
}

//extint0
#endif	//0

#if 0
void (* _int0_isrptr) (void)=empty_handler;

void _ISR_PSV _INT0Interrupt(void) {
	IFS0bits.INT0IF = 0;				//clera the flag
	_int0_isrptr();						//run the isr
}

void int0Init(void) {
	//INT02RP();						//map int0_pin - int0 cannot be remapped
	_int0_isrptr = empty_handler;		//initialize int isr ptr
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT0EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int0AttachISR(void (*isrptr) (void)) {
	_int0_isrptr = isrptr;
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint1
void (* _int1_isrptr) (void)=empty_handler;

void _ISR_PSV _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;				//clera the flag
	_int1_isrptr();						//run the isr
}

void int1Init(void) {
	INT12RP();							//map int1_pin
	_int1_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT1EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int1AttachISR(void (*isrptr) (void)) {
	_int1_isrptr = isrptr;
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint2
void (* _int2_isrptr) (void)=empty_handler;

void _ISR_PSV _INT2Interrupt(void) {
	IFS1bits.INT2IF = 0;				//clera the flag
	_int2_isrptr();						//run the isr
}

void int2Init(void) {
	INT22RP();							//map int2_pin
	_int2_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT2EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int2AttachISR(void (*isrptr) (void)) {
	_int2_isrptr = isrptr;
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}


//end extint
#endif	//0

//spi
//rest spi1
void spi1Init(uint32_t br) {
	uint32_t tmp;
	RCC->APBENR2 |= RCC_APBENR2_SPI1EN;	//1->enable the module
	
	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	//master only, clock polarity: 0 at idle, clock phase: capture on first edge
	SPI1->CR1 = SPI1->CR2 = SPI1->SR = 0;	//reset the spi module
	for (tmp = 0; tmp < 7; tmp++)
		if ((F_PCLK >> (tmp+1)) <= br) break;
	SPI1->CR1 = (SPI1->CR1 &~SPI_CR1_BR) | ((tmp << 3) & SPI_CR1_BR);
	SPI1->CR1 |= SPI_CR1_MSTR_Pos;		//1->master mode, 0->slave mode
	SPI1->CR2 = (SPI1->CR2 &~SPI_CR2_DS) | ((0x07<< 8) & SPI_CR2_DS);	//8bit data length: default
	SPI1->CR2 &=~SPI_CR2_TXEIE;			//0->disable tx empty interrupt
	SPI1->CR2 &=~SPI_CR2_RXNEIE;		//0->disable rx not empty  interrupt
	SPI1->CR2 &=~SPI_CR2_ERRIE;			//0->disable error interrupt
	//SPI1->CR2 &=~SPI_CR2_FRF;			//0->motorolla frame mode, 1->TI frame mode
	//SPI1->CR2 &=~SPI_CR2_NSSP;			//0->no nss pin mgmt
	SPI1->CR2 &=~SPI_CR2_SSOE;			//0->SS output disabled
	//IFS0bits.SPI1IF = 0;				//0->reset the flag
	//IEC0bits.SPI1IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	SPI1->CR1 |= SPI_CR1_SPE;			//1->enable the module, 0->disable the module

	//map the pins
#if defined(NSS1toPIN)
	SPI1->CR2 |= SPI_CR2_NSSP;			//1->with nss pin mgmt
	NSS1toPIN();
#endif
#if defined(SCK1toPIN)
	SCK1toPIN();
#endif
#if defined(MOSI1toPIN)
	MOSI1toPIN();
#endif
#if defined(MISO1toPIN)
	MISO1toPIN();
#endif

}

#if defined(SPI2)
//rest spi2
void spi2Init(uint32_t br) {
	uint32_t tmp;
	RCC->APBENR1 |= RCC_APBENR1_SPI2EN;	//1->enable the module
	
	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	//master only, clock polarity: 0 at idle, clock phase: capture on first edge
	SPI2->CR1 = SPI2->CR2 = SPI2->SR = 0;	//reset the spi module
	for (tmp = 0; tmp < 7; tmp++)
		if ((F_PCLK >> (tmp+1)) <= br) break;
	SPI2->CR1 = (SPI2->CR1 &~SPI_CR1_BR) | ((tmp << 3) & SPI_CR1_BR);
	SPI2->CR1 |= SPI_CR1_SSM;			//1->master mode, 0->slave mode
	SPI2->CR2 = (SPI2->CR2 &~SPI_CR2_DS) | ((0x07<< 8) & SPI_CR2_DS);	//8bit data length: default
	SPI2->CR2 &=~SPI_CR2_TXEIE;			//0->disable tx empty interrupt
	SPI2->CR2 &=~SPI_CR2_RXNEIE;		//0->disable rx not empty  interrupt
	SPI2->CR2 &=~SPI_CR2_ERRIE;			//0->disable error interrupt
	//SPI2->CR2 &=~SPI_CR2_FRF;			//0->motorolla frame mode, 1->TI frame mode
	//SPI2->CR2 &=~SPI_CR2_NSSP;			//0->no nss pin mgmt
	SPI2->CR2 &=~SPI_CR2_SSOE;			//0->SS output disabled
	//IFS0bits.SPI2IF = 0;				//0->reset the flag
	//IEC0bits.SPI2IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	SPI2->CR1 |= SPI_CR1_SPE;			//1->enable the module, 0->disable the module

	//map the pins
#if defined(NSS2toPIN)
	SPI2->CR2 |= SPI_CR2_NSSP;			//1->with nss pin mgmt
	NSS2toPIN();
#endif
#if defined(SCK2toPIN)
	SCK2toPIN();
#endif
#if defined(MOSI2toPIN)
	MOSI2toPIN();
#endif
#if defined(MISO2toPIN)
	MISO2toPIN();
#endif

}

#endif 	//spi2
//end spi

//i2c
//generic i2c initialization
void i2cInit(I2C_TypeDef *I2Cx, uint32_t br) {
	//configure i2c
	//I2Cx->CR1 = I2Cx->CR2 = I2Cx->SR = 0;		//initialize the registers
	I2Cx->CR1 = 0;								//reset state
	//need to figure out timing parameters
	I2Cx->CR2 = APBClock() / 1000000ul;			//4->4Mhz, 36->36Mhz, 48->48Mhz
	if (I2Cx->CR2 < 4) I2Cx->CR2 = 4;			//4 is minimum
	if (I2Cx->CR2 >48) I2Cx->CR2 =48;			//48 is max
	
	I2Cx->SR2|= I2C_SR2_MSL;					//1->master, 0->slave
	
	//enable the i2c
	//I2Cx->CR1 |= I2C_CR1_PE;					//1->enable the i2c module
}

//i2c1 initialization
void i2c1Init(uint32_t br) {
	RCC->APBENR1 |= RCC_APBENR1_I2CEN;			//1->enable clock

	//configure i2c1
	i2cInit(I2C1, br);

	//enable the i2c
	I2C1->CR1 |= I2C_CR1_PE;					//1->enable the i2c module
	
	//scl/sda pins must exist!
#if defined(SCL1toPIN)
	SCL1toPIN();								//enable scl pin - you must have SCL pins
#endif

#if defined(SDA1toPIN)
	SDA1toPIN();								//enable sda pin - you must have SDA pins
#endif
}

//send multiple bytes
int16_t i2c1Writes(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToWrite) {  
	while (i2c1Busy()) continue;				//wait for the prior transmission to end
	
	i2c1Start();								//send a start condition
	i2c1WriteAddr(DeviceAddr | I2C_CMD_WRITE);	//write device address
	i2c1Write(RegAddr);							//WRITE_REG the register address
	//write multiple bytes
	while(NumByteToWrite--) 
		i2c1Write(*pBuffer++);					//send data
	i2c1Stop();  								//generate stop condition
	i2c1Ack();									//enable ack
	return 0;
}

//read multiple bytes
int16_t i2c1Reads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead) {  
	while (i2c1Busy()) continue;				//wait for the prior transmission to end
	
	i2c1Start();								//send a start condition
	i2c1WriteAddr(DeviceAddr | I2C_CMD_WRITE);	//write device address
	i2c1Write(RegAddr);							//WRITE_REG the register address
	
	//send the start condition again
	i2c1Start();								//send a start condition
	i2c1WriteAddr(DeviceAddr | I2C_CMD_READ);	//write device address
	I2C1->SR1; I2C1->SR2;						//clear the status
	//read multiple bytes
	while(NumByteToRead) {
		if (NumByteToRead==1) {i2c1NoAck(); i2c1Stop();}
		if (i2c1Avail()) {*pBuffer++=I2C1->DR; NumByteToRead--;}
	}
	i2c1Ack();									//enable ack
	return 0;
}

//end i2c


#if defined(RTC)
//rtcc
//global variables
static void (* _isrptr_ovf)(void)=empty_handler;			//rtc overflow flag
static void (* _isrptr_alr)(void)=empty_handler;			//rtc alarm flag
static void (* _isrptr_sec)(void)=empty_handler;			//rtc second flag

//rtc alarm isr
void RTC_IRQHandler(void) {
	//overflow isr
	if (RTC->CRL & RTC_CRL_OWF) {							//overflow flag is set
		RTC->CRL &=~RTC_CRL_OWF;							//clear the overflow flag
		_isrptr_ovf();										//execute user handler
	}

	//alarm isr - can be configured through EXTI line 17 as well
	if (RTC->CRL & RTC_CRL_ALRF) {							//overflow flag is set
		RTC->CRL &=~RTC_CRL_ALRF;							//clear the overflow flag
		_isrptr_alr();										//execute user handler
	}

	//second isr
	if (RTC->CRL & RTC_CRL_SECF) {							//overflow flag is set
		RTC->CRL &=~RTC_CRL_SECF;							//clear the overflow flag
		_isrptr_sec();										//execute user handler
	}
}

//reset the rtc
//ps = prescaler
//for 32768Khz, 0x7fff for 1-second
//pr = period for alarm
//alarm isr is disabled
//rtcc_clk is clock source:
//RCC_BDCR_RTCSEL_LSI, RCC_BDCR_RTCSEL_LSE and RCC_BDCR_RTCSEL_HSE (HSE/128)
void RTCInit(uint32_t rtcc_clk, uint32_t ps) {
	_isrptr_alr = _isrptr_ovf = _isrptr_sec = empty_handler;	//set default isr_ptr value

	RCC->APBENR1 |= RCC_APBENR1_RTCAPBEN |		//1->enable rtc, 0->disable rtc  clock
					RCC_APBENR1_PWREN |			//1->enable low power control block
					0x00;
	
	PWR->CR1     |= PWR_CR1_DBP;							// enable access to RTC, BDC registers
	//reset backup domain: BDRST bit goes through a 1->0 transition -> needed to change rtc clock selection
	RCC->BDCR    |= RCC_BDCR_BDRST; RCC->BDCR    &=~RCC_BDCR_BDRST;		  //1->0: reset the backup domain
	switch (rtcc_clk) {
	case RCC_BDCR_RTCSEL_LSI:		//lsi
		//enable LSI -> only if LSI is used as a clock source
		RCC->CSR |= RCC_CSR_LSION;							//1->turn on LSI, 0->turn off LSI
		while ((RCC->CSR & RCC_CSR_LSIRDY) == 0) continue;	//1->LSI ready, 0->LSI not ready
		RCC->BDCR |= (RCC_BDCR_RTCSEL_LSI | RCC_BDCR_RTCEN);          // set RTC clock source(LSI/LSE/HSE), enable RTC clock
		break;
	case RCC_BDCR_RTCSEL_LSE:		//lse
		//enable LSE and wait for it to be ready
		RCC->BDCR |= RCC_BDCR_LSEON;						//1->turn on LSE, 0->turn off LSE
		while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0) continue;	//1->LSE ready, 0->LSE not ready
		RCC->BDCR |= (RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN);          // set RTC clock source(LSI/LSE/HSE), enable RTC clock
		break;
	case RCC_BDCR_RTCSEL_HSE:		//hse/128
		//enable HSE ->
		RCC->CR |= RCC_CR_HSEON;							//1->turn on LSI, 0->turn off LSI
		while ((RCC->CR & RCC_CR_HSERDY) == 0) continue;	//1->HSE ready, 0->HSE not ready
		RCC->BDCR |= (RCC_BDCR_RTCSEL_HSE | RCC_BDCR_RTCEN);          // set RTC clock source(LSI/LSE/HSE), enable RTC clock
		break;
	}
	RTC->CRL &= ~RTC_CRL_RSF;                  			//Clear registers Synchronized Flag
	while((RTC->CRL & RTC_CRL_RSF) == 0) continue;      //Wait for the RSF bit in RTC_CRL to be set by hardware
	//configuration sequence
	//1. wait for rtoff goes high
	while (!RTC->CRL & RTC_CRL_RTOFF) continue;
	//2. set cnf to enter configuration mode
	RTC->CRL |= RTC_CRL_CNF;
	//3. write to 1 or more rtc registers
	PWR->CR1 |= PWR_CR1_DBP;					//1->set dbp bit, 0->clear dbp bit
	ps = ps - 1;								//update the prescaler
	RTC->PRLL = ps; RTC->PRLH = ps >> 16;		//setup hte prescaler. can only write to this if PWR_CR1.DBP is 1
	RCC->BDCR = (RCC->BDCR &~RCC_BDCR_RTCSEL) | (rtcc_clk & RCC_BDCR_RTCSEL);	//can only write to it if pwr_cr1.dbp is 1
	PWR->CR1 &=~PWR_CR1_DBP;					//1->set dbp bit, 0->clear dbp bit
	//4. clear cnf to exit configuration mode
	RTC->CRL &=~RTC_CRL_CNF;
	//5. poll rtoff until it changes to 1
	while (!RTC->CRL & RTC_CRL_RTOFF) continue;
	//enable rtc
	//RCC->BDCR |= RCC_BDCR_RTCEN;				//1->enable rtc, 0->disable rtc
}

//install rtc overflow isr
void RTCOVFAttachISRF(void (*isr_ptr)(void)) {
	_isrptr_ovf = isr_ptr;						//install user handler
	//clear the flags
	RTC->CRL &=~RTC_CRL_OWF;					//clear the overflow flags
	RTC->CRH |= RTC_CRH_OWIE;					//1->enable overflow interrupt
	NVIC_EnableIRQ(RTC_IRQn);					//enable rtc handler
}

//install rtc alarm isr
void RTCALRAttachISR(void (*isr_ptr)(void)) {
	_isrptr_alr = isr_ptr;						//install user handler
	//clear the flags
	RTC->CRL &=~RTC_CRL_ALRF;					//clear the overflow flags
	RTC->CRH |= RTC_CRH_ALRIE;					//1->enable overflow interrupt
	NVIC_EnableIRQ(RTC_IRQn);					//enable rtc handler
}

//install rtc second isr
void RTCSECAttachISR(void (*isr_ptr)(void)) {
	_isrptr_sec = isr_ptr;						//install user handler
	//clear the flags
	RTC->CRL &=~RTC_CRL_SECF;					//clear the overflow flags
	RTC->CRH |= RTC_CRH_SECIE;					//1->enable overflow interrupt
	NVIC_EnableIRQ(RTC_IRQn);					//enable rtc handler
}

//set rtc counter
uint32_t RTCSetCNT(uint32_t val) {
	PWR->CR1      |= PWR_CR1_DBP;							// enable write access to RTC, BDC registers
	//unlocking sequence needed for CRL, CNT and ALR registers
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0) continue;	//wait for RTOFF value to 1 -> prior operations have finished
	RTC->CRL |= RTC_CRL_CNF;					//1->set CNF to enter configuration mode, 0->exit configuration mode
	//write to one or more RTC registers
	RTC->CNTH = val >> 16; RTC->CNTL = val;		//set up the counter
	//RTC->ALRH = alr>> 16; RTC->ALRL = alr;	//set up the alarm
	//locking sequency
	RTC->CRL &=~RTC_CRL_CNF;					//1->set CNF to enter configuration mode, 0->exit configuration mode
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0) continue;	//wait for RTOFF value to 1 -> prior operations have finished
	//PWR->CR      &=~PWR_CR_DBP;							// disable access to RTC, BDC registers
	return val;
}

//read rtc counter
uint32_t RTCGetCNT(void) {
	uint32_t tmp;
	uint16_t lsb;

	RTC->CRL &= ~RTC_CRL_RSF;                  			//Clear registers Synchronized Flag
	//while((RTC->CRL & RTC_CRL_RSF) == 0) continue;      //Wait for the RSF bit in RTC_CRL to be set by hardware
	do {
		tmp = RTC->CNTH;
		lsb = RTC->CNTL;
	} while (tmp != RTC->CNTH);

	return (tmp << 16) | lsb;
}

//set rtc alarm
uint32_t RTCSetALR(uint32_t val) {
	PWR->CR1      |= PWR_CR1_DBP;							// enable write access to RTC, BDC registers
	//unlocking sequence needed for CRL, CNT and ALR registers
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0) continue;	//wait for RTOFF value to 1 -> prior operations have finished
	RTC->CRL |= RTC_CRL_CNF;					//1->set CNF to enter configuration mode, 0->exit configuration mode
	//write to one or more RTC registers
	//RTC->CNTH = val >> 16; RTC->CNTL = val;	//set up the counter
	RTC->ALRH = val>> 16; RTC->ALRL = val;		//set up the alarm
	//locking sequency
	RTC->CRL &=~RTC_CRL_CNF;					//1->set CNF to enter configuration mode, 0->exit configuration mode
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0) continue;	//wait for RTOFF value to 1 -> prior operations have finished
	//PWR->CR      &=~PWR_CR_DBP;							// disable access to RTC, BDC registers
	return val;
}

//read rtc alarm
uint32_t RTCGetALR(void) {
	uint32_t tmp;
	uint16_t lsb;

	RTC->CRL &= ~RTC_CRL_RSF;                  			//Clear registers Synchronized Flag
	while((RTC->CRL & RTC_CRL_RSF) == 0) continue;      //Wait for the RSF bit in RTC_CRL to be set by hardware
	do {
		tmp = RTC->ALRH;
		lsb = RTC->ALRL;
	} while (tmp != RTC->ALRH);

	return (tmp << 16) | lsb;
}

//read rtc
time_t RTC2time(time_t *t) {
	uint32_t tmp=RTCGetCNT();


	if (t==NULL) return (tmp==0)?(-1):tmp; 			//just return seconds since epoch time if pointer is NULL
	return *t=(tmp==0)?(-1):tmp;						//change pointed value + return seconds since epoch time
}

//initialize rtc with time_t
time_t time2RTC(time_t t) {
	t=(t<946684800ul)?946684800ul:t;					//minimum time is 1/1/2000 for the hardware rtc
	RTCSetCNT(t);
	return t;
}

//read prll and prlh
uint32_t RTCGetPRL(void) {
	RTC->CRL &=~RTC_CRL_RSF; while ((RTC->CRL & RTC_CRL_RSF) == 0) continue;	//wait for RSF to be set (RTC in sync)
	return ((RTC->PRLH << 16) | RTC->PRLL) + 1;
}

//read divl and divh
uint32_t RTCGetDIV(void) {
	RTC->CRL &=~RTC_CRL_RSF; while ((RTC->CRL & RTC_CRL_RSF) == 0) continue;	//wait for RSF to be set (RTC in sync)
	return (RTC->DIVH << 16) | RTC->DIVL;
}

//returns:
//0: if 1st half second
//1: if 2nd half second
uint8_t RTCHalfsec(void) {
	return (RTCGetDIV() < RTCGetPRL() / 2)?0:1;
}

//set calibration register
void RTCSetCal(uint8_t cal) {
	//PWR->CR      |= PWR_CR_DBP;							// enable write access to RTC, BDC registers
	//unlocking sequence needed for CRL, CNT and ALR registers
	RTC->BKP_RTCCR = (RTC->BKP_RTCCR &~BKP_RTCCR_CAL) | (cal & BKP_RTCCR_CAL);
	//PWR->CR      &=~PWR_CR_DBP;							// disable access to RTC, BDC registers
}
//end rtcc
#endif	//rtc

//software rtc
volatile sRTC_TypeDef sRTC={-1, 0, 0, 0};		//software RTC

//initialize software counter
void sRTCInit(void) {					//calibration from -128ppm to +128ppm, if sRTC_RATE = 1M
	//sRTC.tick_rate=sRTC_RATE;
	sRTC.time=-1;						//reset counter: -1=uninitiated timer
	sRTC.tick=0;						//reset the tick counter
	sRTC.cal=0;							//initialize calibration
	sRTC.halfsec=0;						//0->1st half sec, 1->2nd half sec
}

//initialize the calibration
void sRTCSetCal(int16_t cal) {
	sRTC.cal=cal;
}

//increment sRTC time - called sRTC_CALLRATE per second
void sRTCISR(void) {
	uint32_t tks=ticks();				//timestamp tks
	if (tks - sRTC.tick > F_CPU) {		//if enough time has passed
		sRTC.tick += F_CPU;				//advance tick counter
		sRTC.time += 1;					//update time
	}
	sRTC.halfsec = (tks - sRTC.tick < F_CPU / 2)?0:1;
}

//read sRTC second counter
time_t sRTC2time(time_t *t) {
	if (t==NULL) return sRTC.time;
	return *t=sRTC.time;
}

//set sRTC second counter - used for initialization
//returns -1 if uninitialized
time_t time2sRTC(time_t t) {
	t=(t<946684800ul)?946684800ul:t;					//minimum time is 1/1/2000 for the hardware rtc
	return sRTC.time=t;
}

//read rtc tick
uint32_t sRTCTick(void) {
	return sRTC.tick;
}

//return half sec
//0: first half of a sec
//1: 2nd half of a sec
uint8_t sRTCHalfsec(void) {
	return sRTC.halfsec;
}

//end software rtc

//exti
//global variables
static void (* _exti0_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti1_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti2_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti3_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti4_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti5_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti6_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti7_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti8_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti9_isrptr )(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti10_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti11_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti12_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti13_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti14_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti15_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default

//EXTI0-1 ISR
void EXTI0_1_IRQHandler(void) {
	//clear the flag and execute user isr
/*	switch (EXTI->PR) {
		case (1<<0): EXTI->PR |= (1<<0); _exti0_isrptr(); break;
		case (1<<1): EXTI->PR |= (1<<1); _exti1_isrptr(); break;
	}
*/
	if (EXTI->PR & (1<<0)) {EXTI->PR |= (1<<0); _exti0_isrptr();}
	if (EXTI->PR & (1<<1)) {EXTI->PR |= (1<<1); _exti1_isrptr();}

}

//EXTI2-3 ISR
void EXTI2_3_IRQHandler(void) {
	//clear the flag and execute user isr
/*	switch (EXTI->PR) {
		case (1<<2): EXTI->PR |= (1<<2); _exti2_isrptr(); break;
		case (1<<3): EXTI->PR |= (1<<3); _exti3_isrptr(); break;
	}
*/
	if (EXTI->PR & (1<<2)) {EXTI->PR |= (1<<2); _exti2_isrptr();}
	if (EXTI->PR & (1<<3)) {EXTI->PR |= (1<<3); _exti3_isrptr();}
}

//EXTI15-10 ISR
void EXTI4_15_IRQHandler(void) {
	//clear the flag and execute user isr
/*
	switch (EXTI->PR) {
		case (1<< 4): EXTI->PR |= (1<< 4); _exti4_isrptr(); break;
		case (1<< 5): EXTI->PR |= (1<< 5); _exti5_isrptr(); break;
		case (1<< 6): EXTI->PR |= (1<< 6); _exti6_isrptr(); break;
		case (1<< 7): EXTI->PR |= (1<< 7); _exti7_isrptr(); break;
		case (1<< 8): EXTI->PR |= (1<< 8); _exti8_isrptr(); break;
		case (1<< 9): EXTI->PR |= (1<< 9); _exti9_isrptr(); break;
		case (1<<10): EXTI->PR |= (1<<10); _exti10_isrptr(); break;
		case (1<<11): EXTI->PR |= (1<<11); _exti11_isrptr(); break;
		case (1<<12): EXTI->PR |= (1<<12); _exti12_isrptr(); break;
		case (1<<13): EXTI->PR |= (1<<13); _exti13_isrptr(); break;
		case (1<<14): EXTI->PR |= (1<<14); _exti14_isrptr(); break;
		case (1<<15): EXTI->PR |= (1<<15); _exti15_isrptr(); break;
	}
*/
	if (EXTI->PR & (1<< 4)) {EXTI->PR |= (1<< 4); _exti4_isrptr();}
	if (EXTI->PR & (1<< 5)) {EXTI->PR |= (1<< 5); _exti5_isrptr();}
	if (EXTI->PR & (1<< 6)) {EXTI->PR |= (1<< 6); _exti6_isrptr();}
	if (EXTI->PR & (1<< 7)) {EXTI->PR |= (1<< 7); _exti7_isrptr();}
	if (EXTI->PR & (1<< 8)) {EXTI->PR |= (1<< 8); _exti8_isrptr();}
	if (EXTI->PR & (1<< 9)) {EXTI->PR |= (1<< 9); _exti9_isrptr();}
	if (EXTI->PR & (1<<10)) {EXTI->PR |= (1<<10); _exti10_isrptr();}
	if (EXTI->PR & (1<<11)) {EXTI->PR |= (1<<11); _exti11_isrptr();}
	if (EXTI->PR & (1<<12)) {EXTI->PR |= (1<<12); _exti12_isrptr();}
	if (EXTI->PR & (1<<13)) {EXTI->PR |= (1<<13); _exti13_isrptr();}
	if (EXTI->PR & (1<<14)) {EXTI->PR |= (1<<14); _exti14_isrptr();}
	if (EXTI->PR & (1<<15)) {EXTI->PR |= (1<<15); _exti15_isrptr();}
}

//global defines

//global variables
//initialize the exti
//pin: one pin only
//gpio: GPIOA..GPIOG
//void extiInit(GPIO_TypeDef * gpio, uint16_t pin, uint8_t edge) {
void extiInit(PIN_TypeDef PINx, uint8_t edge) {
	uint16_t tmp=0;
	GPIO_TypeDef *gpio;
	uint16_t pin;
	
	gpio=GPIO_PinDef[PINx].gpio;
	pin =GPIO_PinDef[PINx].mask;
	//enable afio
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	//enable the clock
	switch ((uint32_t) gpio) {
#if defined(GPIOA)
		case (uint32_t) GPIOA: RCC->IOPENR |= RCC_IOPENR_GPIOAEN_Pos; tmp = 0x00; break;
#endif
#if defined(GPIOB)
		case (uint32_t) GPIOB: RCC->IOPENR |= RCC_IOPENR_GPIOBEN_Pos; tmp = 0x01; break;
#endif
#if defined(GPIOC)
		case (uint32_t) GPIOC: RCC->IOPENR |= RCC_IOPENR_GPIOCEN_Pos; tmp = 0x02; break;
#endif
#if defined(GPIOD)
		case (uint32_t) GPIOD: RCC->IOPENR |= RCC_IOPENR_GPIODEN_Pos; tmp = 0x03; break;
#endif
#if defined(GPIOE)
		case (uint32_t) GPIOE: RCC->IOPENR |= RCC_IOPENR_GPIOEEN_Pos; tmp = 0x04; break;
#endif
#if defined(GPIOF)
		case (uint32_t) GPIOF: RCC->IOPENR |= RCC_IOPENR_GPIOFEN_Pos; tmp = 0x05; break;
#endif
#if defined(GPIOG)
		case (uint32_t) GPIOG: RCC->IOPENR |= RCC_IOPENR_GPIOGEN_Pos; tmp = 0x06; break;
#endif
	}
	//configure exti edges
	//1->falling/rising edge enabled, 0->falling/rising edge disabled
	switch (edge) {
		case EXTI_FALLING: 	EXTI->FTSR |= pin; EXTI->RTSR &=~pin; break;
		case EXTI_RISING: 	EXTI->FTSR &=~pin; EXTI->RTSR |= pin; break;
		case EXTI_BOTH: 	EXTI->FTSR |= pin; EXTI->RTSR |= pin; break;
	}
	//configure the port
	switch (pin) {
		case (1<< 0): EXTI->EXTICR[0] = (EXTI->EXTICR[0] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 1): EXTI->EXTICR[0] = (EXTI->EXTICR[0] &~0x00f0) | ((tmp) <<  4); break;
		case (1<< 2): EXTI->EXTICR[0] = (EXTI->EXTICR[0] &~0x0f00) | ((tmp) <<  8); break;
		case (1<< 3): EXTI->EXTICR[0] = (EXTI->EXTICR[0] &~0xf000) | ((tmp) << 12); break;
		case (1<< 4): EXTI->EXTICR[1] = (EXTI->EXTICR[1] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 5): EXTI->EXTICR[1] = (EXTI->EXTICR[1] &~0x00f0) | ((tmp) <<  4); break;
		case (1<< 6): EXTI->EXTICR[1] = (EXTI->EXTICR[1] &~0x0f00) | ((tmp) <<  8); break;
		case (1<< 7): EXTI->EXTICR[1] = (EXTI->EXTICR[1] &~0xf000) | ((tmp) << 12); break;
		case (1<< 8): EXTI->EXTICR[2] = (EXTI->EXTICR[2] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 9): EXTI->EXTICR[2] = (EXTI->EXTICR[2] &~0x00f0) | ((tmp) <<  4); break;
		case (1<<10): EXTI->EXTICR[2] = (EXTI->EXTICR[2] &~0x0f00) | ((tmp) <<  8); break;
		case (1<<11): EXTI->EXTICR[2] = (EXTI->EXTICR[2] &~0xf000) | ((tmp) << 12); break;
		//case (1<<12): EXTI->EXTICR[3] = (EXTI->EXTICR[3] &~0x000f) | ((tmp) <<  0); break;
		//case (1<<13): EXTI->EXTICR[3] = (EXTI->EXTICR[3] &~0x00f0) | ((tmp) <<  4); break;
		//case (1<<14): EXTI->EXTICR[3] = (EXTI->EXTICR[3] &~0x0f00) | ((tmp) <<  8); break;
		//case (1<<15): EXTI->EXTICR[3] = (EXTI->EXTICR[3] &~0xf000) | ((tmp) << 12); break;
	};
}

//install user handler
//void extiAttachISR(uint16_t pin, void (*isr_ptr)(void)) {
void extiAttachISR(PIN_TypeDef PINx, void (*isr_ptr)(void)) {
	GPIO_TypeDef *gpio;
	uint16_t pin;
	
	gpio=GPIO_PinDef[PINx].gpio;
	pin =GPIO_PinDef[PINx].mask;
	//clear the flag
	EXTI->PR |= pin;						//1->clear the flag
	//enable the interrupt
	EXTI->IMR |= pin;							//1->enable the interrupt, 0->disable the interrupt
	EXTI->EMR |= pin;							//1->enable the event trigger, 0->disable the event trigger

	//enable nvic
	switch (pin) {
		case (1<< 0): NVIC_EnableIRQ(EXTI0_1_IRQn ); _exti0_isrptr  = isr_ptr; break;
		case (1<< 1): NVIC_EnableIRQ(EXTI0_1_IRQn ); _exti1_isrptr  = isr_ptr; break;
		case (1<< 2): NVIC_EnableIRQ(EXTI2_3_IRQn ); _exti2_isrptr  = isr_ptr; break;
		case (1<< 3): NVIC_EnableIRQ(EXTI2_3_IRQn ); _exti3_isrptr  = isr_ptr; break;
		case (1<< 4): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti4_isrptr  = isr_ptr; break;
		case (1<< 5): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti5_isrptr  = isr_ptr; break;
		case (1<< 6): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti6_isrptr  = isr_ptr; break;
		case (1<< 7): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti7_isrptr  = isr_ptr; break;
		case (1<< 8): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti8_isrptr  = isr_ptr; break;
		case (1<< 9): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti9_isrptr  = isr_ptr; break;
		case (1<<10): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti10_isrptr = isr_ptr; break;
		case (1<<11): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti11_isrptr = isr_ptr; break;
		case (1<<12): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti12_isrptr = isr_ptr; break;
		case (1<<13): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti13_isrptr = isr_ptr; break;
		case (1<<14): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti14_isrptr = isr_ptr; break;
		case (1<<15): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti15_isrptr = isr_ptr; break;
	}
}

//end exti

//analog random number - generate true random number with analog read
//analog random
//generate 4-bit random numbers
uint8_t arand4(void) {
	uint8_t tmp=arand1();

	//tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	return tmp;
}

//generate 8-bit random number
uint8_t arand8(void) {
	return (arand4() << 4) | arand4();
}

//return 16-bit random number
uint16_t arand16(void) {
	return ((uint16_t) arand8() << 8) | arand8();
}


//return 32-bit random number
uint32_t arand32(void) {
	return ((uint32_t) arand16() << 16) | arand16();
}
//end analog random
//end analog random number

//circular buffer
//global defines
#define cBUFF_SIZE_MASK 	(cBUFF_SIZE-1U)		//mask

//global variables
//reset the cbuffer
void cbInit(CB_TypeDef *cbuffer) {
	cbuffer->writeIndex = cbuffer->readIndex = -1;
}

//returns true if cbuffer is empty
char cbEmpty(CB_TypeDef *cbuffer) {
	return (cbuffer->writeIndex & cBUFF_SIZE_MASK) == (cbuffer->readIndex & cBUFF_SIZE_MASK);
}

//returns true if cbuffer is full;
char cbFull(CB_TypeDef *cbuffer) {
	return ((cbuffer->writeIndex + 1) & cBUFF_SIZE_MASK) == (cbuffer->readIndex & cBUFF_SIZE_MASK);
}

//write to the buffer - needs to test if the buffer is full before writing to it
cbuffer_t cbWrite(CB_TypeDef *cbuffer, cbuffer_t dat) {
	cbuffer->buffer[(++cbuffer->writeIndex) & cBUFF_SIZE_MASK] = dat;
	return dat;
}

//read from the buffer - needs to test if the buffer is empty before reading from it
cbuffer_t cbRead(CB_TypeDef *cbuffer) {
	return cbuffer->buffer[(++cbuffer->readIndex) & cBUFF_SIZE_MASK];
}

//write string to CB
void str2cb(CB_TypeDef *cb, char *str) {
	while (!cbFull(cb) && (*str)) cbWrite(cb, *str++);
}

//write CB to string
void cb2str(char *str, CB_TypeDef *cb) {
	while (!cbEmpty(cb)) *str++=cbRead(cb);
}

//end circular buffer

