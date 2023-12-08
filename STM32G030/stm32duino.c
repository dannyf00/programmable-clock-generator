#include "stm32duino.h"				//stm32duino

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

//clock mgmt
//switch system clock
//0b000->HSI
//0b001->HSE
//0b010->PLLRCLK
//0b011->LSI
//0b100->LSE
//need to confirm LSI/LSE
void SystemCoreClockConfig(uint8_t clk) {
	switch (clk) {
		case 0b000: RCC->CR  |= RCC_CR_HSION;   while ((RCC->CR  & RCC_CR_HSIRDY  )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on HSI and wait for it to stablize
		case 0b001: RCC->CR  |= RCC_CR_HSEON;   while ((RCC->CR  & RCC_CR_HSERDY  )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on HSe and wait for it to stablizebreak;
		case 0b010: RCC->CR  |= RCC_CR_PLLON;   while ((RCC->CR  & RCC_CR_PLLRDY  )== 0) continue; RCC->PLLCFGR|= RCC_PLLCFGR_PLLREN; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on PLL and wait for it to stablizebreak;
		case 0b011: RCC->CSR |= RCC_CSR_LSION;  while ((RCC->CSR & RCC_CSR_LSIRDY )== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on LSI and wait for it to stablizebreak;
		case 0b100: RCC->BDCR|= RCC_BDCR_LSEON; while ((RCC->BDCR& RCC_BDCR_LSERDY)== 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | clk; break;	//turn on LSE and wait for it to stablizebreak;
	}
	SystemCoreClockUpdate();								//update the clock
	if (clk == 0b010) SystemCoreClock /= 2;					//pll seems to be running at 2:1
}
//set pll
//return Fvco: 96..344Mhz (volt scalling range 1)
//Fvco=Fpllin * N / (M + 1)
//N: 8..86, M = 0..7
void pllSet(uint8_t N, uint8_t M) {
	uint32_t tmp;
	//retard flash latency to the highest
	FLASH->ACR = (FLASH->ACR &~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2;

	//switch to HSI to be safe
	RCC->CR |= RCC_CR_HSION; while ((RCC->CR & RCC_CR_HSIRDY) == 0) continue; RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | 0b000; 
	//DISABLE the pll first - pll parameters cannot be changed with pll on
	RCC->CR &=~RCC_CR_PLLON; 
	while (RCC->CR & RCC_CR_PLLRDY) continue; 				//wait for rcc to be powered down
	//RCC->PLLCFGR&=~RCC_PLLCFGR_PLLREN;					//disable pllr
	RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLM) | ((M<<RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM);
	RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLN) | ((N<<RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN);
	RCC->CR |= RCC_CR_PLLON; 								//ENABLE pll
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) continue; 		//wait for rcc to be powered up
	RCC->PLLCFGR|= RCC_PLLCFGR_PLLREN;						//enable pllr

	//adjust flash latency: 
	//0: less than 24Mhz, 1: 24 - 48Mhz, 2: otherwise
	tmp = AHBClock();
	if (tmp < 24000000ul) FLASH->ACR = (FLASH->ACR &~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
	else if (tmp < 48000000ul) FLASH->ACR = (FLASH->ACR &~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1;
}

//ahb clock
uint32_t AHBClock(void) {
	uint32_t tmp;
	
	tmp = F_SYSCLK;					//HCLK derived from SYSCLK
	switch ((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos) {
		case 0 ... 7: 	tmp /=  1;  break;
		case 8:			tmp /=  2;  break;
		case 9: 		tmp /=  4;  break;
		case 10:		tmp /=  8;  break;
		case 11:		tmp /=  16; break;
		case 12:		tmp /=  32; break;
		case 13:		tmp /=  64; break;
		case 14:		tmp /= 128; break;
		case 15:		tmp /= 256; break;
		case 16:		tmp /= 512; break;
	}
	return tmp;
}

//apb clock
uint32_t APBClock(void) {
	uint32_t tmp;
	
	tmp = F_HCLK;					//PCLK derived from HCLK
	switch ((RCC->CFGR & RCC_CFGR_PPRE) >> RCC_CFGR_PPRE_Pos) {
		case 0 ... 3: 	tmp /=  1;  break;
		case 4:			tmp /=  2;  break;
		case 5: 		tmp /=  4;  break;
		case 6:			tmp /=  8;  break;
		case 7:			tmp /=  16; break;
	}
	return tmp;
}

//PLL VCO input clock
uint32_t F_PLLIN(void) {
	uint32_t tmp;
	switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
		case 0b00: tmp = 0; break;				//no clock
		case 0b01: tmp =-1; break;				//reserved
		case 0b10: tmp = F_HSI16; break;		//HSI16
		case 0b11: tmp = F_HSE; break;			//HSE
	}
	return tmp;
}

//#define F_UART1		(((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (0<< 0))?F_PCLK:  (((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (1<< 0))?F_SYSCLK:(((RCC->CCIPR & RCC_CCIPR_USART1SEL) == (2<< 0))?F_HSI16:F_LSE)))
uint32_t u1FreqGet(void) {
	uint32_t tmp;
	
	switch (RCC->CCIPR & RCC_CCIPR_USART1SEL) {
		case 0<<0: tmp = F_PCLK; break;
		case 1<<0: tmp = F_SYSCLK;
		case 2<<0: tmp = F_HSI16;
		case 3<<0: tmp = F_LSE;
	}
	return tmp;
}

//reset the mcu
void mcuInit(void) {
	//for debugging purposes
	volatile uint32_t i=1000000ul; while (i--) continue;
	
	//select the clock source
	//or use default clock = 8Mhz FRC
	//SystemCoreClockHSEPLL(RCC_CFGR_PLLMULL12);
	//SystemCoreClockHSE();
	//SystemCoreClockHSI();
	//SystemCoreClockHSIPLL_32Mhz();				//HSIPLL to 32Mhz
	//update SystemCoreClock - it must be the last step before exiting mcuInit
	SystemCoreClockUpdate();

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
//helper functions
uint32_t pinPos(uint32_t mask) {
	uint32_t tmp=0;
	while ((mask & (1ul<<tmp)) == 0) tmp++;
	return tmp;
}

//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
void pinMode(PIN_TypeDef pin, uint8_t mode) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;
	uint8_t pinPosx=pinPos(mask);
	
	switch (mode) {
		case INPUT:				gpio->PUPDR &=0b00<<(2*pinPosx); gpio->PUPDR |= 0b00<<(2*pinPosx); GIO_IN(gpio, mask); break;
		case INPUT_PU:			gpio->PUPDR &=0b00<<(2*pinPosx); gpio->PUPDR |= 0b01<<(2*pinPosx); GIO_IN(gpio, mask); break;
		case INPUT_PD:			gpio->PUPDR &=0b00<<(2*pinPosx); gpio->PUPDR |= 0b10<<(2*pinPosx); GIO_IN(gpio, mask); break;
		case OUTPUT: 			IO_OUTPP(gpio, mask);break;
		case OUTPUT_OD: 		IO_OUTOD(gpio, mask);break;
	}
	//if (mode==OUTPUT) GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	//else GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//set / clear a pin
void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;

	if (val==LOW) GIO_CLR(gpio, mask);	else GIO_SET(gpio, mask);
}

//read a pin
int digitalRead(PIN_TypeDef pin) {
	GPIO_TypeDef *gpio=GPIO_PinDef[pin].gpio;
	uint16_t mask = GPIO_PinDef[pin].mask;

	return (GIO_GET(gpio, mask))?HIGH:LOW;
}


//configure pin for AFIO
void pinAFIO(PIN_TypeDef pin, AFIO_TypeDef AFIOn) {
	uint32_t pos;
	GPIO_TypeDef *gpio;

	gpio=GPIO_PinDef[pin].gpio;
	//mask=GPIO_PinDef[pin].mask;
	pos = pinPos(GPIO_PinDef[pin].mask);
	gpio->MODER = (gpio->MODER &~(/*0b11*/0x03<<(2* pos))) | (/*0b10*/0x02<<(2* pos));
	gpio->OTYPER &=~(1<< pos);
	gpio->OSPEEDR = (gpio->OSPEEDR &~(/*0b11*/0x03<<(2* pos))) | (/*0b01*/0x01<<(2* pos));
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

//generic usart reset routine
void uartInit(USART_TypeDef *USARTx, unsigned long baudrate) {
	uint32_t uartdiv;

    //USARTx->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USARTx->CR1 =	(0<<31) |		//0->disable rxfifo interrupt
					(0<<30) |		//0->disable txfifo interrupt
					(1<<29) |		//1->enable fifo
					(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
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
    				(0<<1) |		//0->uart not able to wake up device in low power mode
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
    USARTx->CR3 =	(0<<29) |		//txfifo threshold. 0->1/8 of buffer, ..., 2->1/2 of buffer, ... 5->completely empty
					(0<<28) |		//1->enable rxftie
					(0<<25) |		//rxfifo threshold. 0->1/8 of buffer, ..., 2->1/2 of buffer, ... 5->completely full
					(0<<24) |		//0->disable complete before guard time interrupt
					(0<<23) |		//1->enable tx threshold interrupt
					(0<<22) |		//0->disable wake up from interrupt
					(0<<20) |		//wake up
					(0<<17) |		//smart card auto retry count
					(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    if (USARTx == USART1) uartdiv = (F_UART1 + baudrate / 2) / baudrate;	//with rounding
	else uartdiv = (F_UART + baudrate / 2) / baudrate;	//with rounding
	//if (uartdiv < 50) USARTx->CR1 |= (1<<15);	//oversample by 8 - to assure 1% baudrate error
    if (USARTx->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(0<<3) |			//bit 3 is always clear
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USARTx->BRR = uartdiv;
    //USARTx->BRR = F_UART / baudrate * ((USARTx->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //USARTx->DR = 0;					//reset the data register
    //USARTx->SR = 0;					//reset the data register

    //enable uart1
    USARTx->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart
}

//generic print to uart buffer
void uPrint(char *uRAM, int32_t dat) {
	uint8_t idx=20;						//buffer index
	uint8_t negFlag=0;					//flag for negative sign: 1->negative, 0->positive

	if (dat<0) {negFlag = 1; dat = -dat;}
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

	//reset usart
	uartInit(USART1, baudrate);

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
	//RX as floating input/AF input, AF1
    //IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    //IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));
#if defined(U1TX2PIN)
	U1TX2PIN();							//activate tx pin
#endif

#if defined(U1RX2PIN)
	U1RX2PIN();							//activate rx pin
#endif

}

void uart1Putch(char ch) {
    //while (!(USART1->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
    while (uart1Busy()) continue;    			//wait for the transmission buffer to be empty
    USART1->TDR = ch;                        	//load the data buffer
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
	    return USART1->RDR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart1Available(void) {
	//return (USART1->SR & USART_SR_TC)?true:false;
	return (USART1->ISR & USART_ISR_RXNE_RXFNE/*USART_ISR_RXNE*/);
}

//test if uart tx is busy
uint16_t uart1Busy(void) {
    return !(USART1->ISR & USART_ISR_TXE_TXFNF/*USART_ISR_TXE*/);    	//return 1 if TX buffer is empty
}

//print to uart1
void u1Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	uPrint(uRAM, dat);					//print to uBuffer
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
	U2TX2PIN();							//activate tx pin
#endif

#if defined(U2RX2PIN)
	U2RX2PIN();							//activate rx pin
#endif

}

void uart2Putch(char ch) {
	   //while (!(USART2->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
	    while (uart2Busy()) continue;    			//wait for the transmission buffer to be empty
	    USART2->TDR = ch;                        	//load the data buffer
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
	    return USART2->RDR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart2Available(void) {
	//return (USART2->SR & USART_SR_TC)?true:false;
	return (USART2->ISR & USART_ISR_RXNE_RXFNE/*USART_ISR_RXNE*/);
}

//test if uart tx is busy
uint16_t uart2Busy(void) {
	   return !(USART2->ISR & USART_ISR_TXE_TXFNF/*USART_ISR_TXE*/);    	//return 1 if TX buffer is empty
}

//print to uart2
void u2Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	uPrint(uRAM, dat);					//print to uBuffer
	uart2Puts(uRAM);					//send a message on uart1
}
#endif		//USART2
//end Serial

//lpuart
//generic set up for lpuart
//generic usart reset routine
void lpuartInit(USART_TypeDef *LPUARTx, unsigned long baudrate) {
	uint32_t uartdiv;

    LPUARTx->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    LPUARTx->CR1 =	(0<<31) |		//0->disable rxfifo interrupt
					(0<<30) |		//0->disable txfifo interrupt
					(1<<29) |		//1->enable fifo
					(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
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
    				(0<<1) |		//0->uart not able to wake up device in low power mode
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    LPUARTx->CR2 = 	(0x00<<28) |	//address of the uart
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
    LPUARTx->CR3 =	(0<<29) |		//txfifo threshold. 0->1/8 of buffer, ..., 2->1/2 of buffer, ... 5->completely empty
					(0<<28) |		//1->enable rxftie
					(0<<25) |		//rxfifo threshold. 0->1/8 of buffer, ..., 2->1/2 of buffer, ... 5->completely full
					(0<<24) |		//0->disable complete before guard time interrupt
					(0<<23) |		//1->enable tx threshold interrupt
					(0<<22) |		//0->disable wake up from interrupt
					(0<<20) |		//wake up
					(0<<17) |		//smart card auto retry count
					(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = 16 * F_LPUART / baudrate * 16;			//256 * f_lpuart/baudrate
    if (LPUARTx->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    LPUARTx->BRR = uartdiv;
    //LPUARTx->BRR = F_UART / baudrate * ((LPUARTx->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //LPUARTx->DR = 0;					//reset the data register
    //LPUARTx->SR = 0;					//reset the data register

    //enable uart1
    LPUARTx->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart
}

//lpuart1
#if defined(LPUART1)
//uart2
//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void lpuart1Init(unsigned long baudrate) {
	uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APBENR1 |= RCC_APBENR1_LPUART1EN;

    //initialize lpuart
	lpuartInit(LPUART1, baudrate);


    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    //IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    //IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));
#if defined(LPU1TX2PIN)
	LPU1TX2PIN();							//activate tx pin
#endif

#if defined(LPU1RX2PIN)
	LPU1RX2PIN();							//activate rx pin
#endif

}

void lpuart1Putch(char ch) {
	   //while (!(LPUART1->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
	    while (lpuart1Busy()) continue;    			//wait for the transmission buffer to be empty
	    LPUART1->TDR = ch;                        	//load the data buffer
	    //while (!(LPUART1->SR & (1<<6)));    		//wait for the transmission to complete
}

void lpuart1Puts(char *str) {
	while (*str) lpuart1Putch(*str++);
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

void lpuart1Putline(char *ln) {
	//USARTWriteString(ln);
	lpuart1Puts(ln);
	//USARTWriteString("\r\n");
	lpuart1Puts((char *)"\r\n");
}

uint8_t lpuart1Getch(void) {
	   //while (!(LPUART1->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
	    return LPUART1->RDR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t lpuart1Available(void) {
	//return (LPUART1->SR & USART_SR_TC)?true:false;
	return (LPUART1->ISR & USART_ISR_RXNE_RXFNE/*USART_ISR_RXNE*/);
}

//test if uart tx is busy
uint16_t lpuart1Busy(void) {
	   return !(LPUART1->ISR & USART_ISR_TXE_TXFNF/*USART_ISR_TXE*/);    	//return 1 if TX buffer is empty
}

//print to uart2
void lpu1Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	uPrint(uRAM, dat);					//print to uBuffer
	lpuart1Puts(uRAM);					//send a message on uart1
}
#endif		//lpUSART
//end lpuart

//generic timer reset routine - to save space
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

	//force an update (for prescaler
	TIMx->EGR |= TIM_EGR_UG;
	
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
	if ((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE)) {TIM1->SR &=~TIM_SR_UIF; _tim1_ovfisrptr();}
}

//isr for timer1 capture / compare
void TIM1_CC_IRQHandler(void) {
	//oc1 portion
	if ((TIM1->SR & TIM_SR_CC1IF) && (TIM1->DIER & TIM_DIER_CC1IE)) {TIM1->SR &=~TIM_SR_CC1IF; if (TIM1->CCER & TIM_CCER_CC1E) _tim1_cc1 = TIM1->CCR1; else TIM1->CCR1 += _tim1_cc1; _tim1_cc1isrptr(); }
	if ((TIM1->SR & TIM_SR_CC2IF) && (TIM1->DIER & TIM_DIER_CC2IE)) {TIM1->SR &=~TIM_SR_CC2IF; if (TIM1->CCER & TIM_CCER_CC2E) _tim1_cc2 = TIM1->CCR2; else TIM1->CCR2 += _tim1_cc2; _tim1_cc2isrptr(); }
	if ((TIM1->SR & TIM_SR_CC3IF) && (TIM1->DIER & TIM_DIER_CC3IE)) {TIM1->SR &=~TIM_SR_CC3IF; if (TIM1->CCER & TIM_CCER_CC3E) _tim1_cc3 = TIM1->CCR3; else TIM1->CCR3 += _tim1_cc3; _tim1_cc3isrptr(); }
	if ((TIM1->SR & TIM_SR_CC4IF) && (TIM1->DIER & TIM_DIER_CC4IE)) {TIM1->SR &=~TIM_SR_CC4IF; if (TIM1->CCER & TIM_CCER_CC4E) _tim1_cc4 = TIM1->CCR4; else TIM1->CCR4 += _tim1_cc4; _tim1_cc4isrptr(); }
}

//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

	//reset timer
	tmrInit(TIM1, ps);
}


//activate the isr handler
void tmr1OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);		//disable irq

	_tim1_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc1 period
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

//set tim1_oc1 period
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

//set tim1_oc1 period
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

//set tim1_oc1 period
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
//end tmr1

#if defined(TIM2)
//tmr2
//global variables
static void (* _tim2_ovfisrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc1isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc2isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc3isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default
static void (* _tim2_cc4isrptr)(void)=empty_handler;				//TIM2_ptr pointing to empty_handler by default

static uint32_t _tim2_cc1=0;				//output compare registers - tmr2 is a 32-bit timer
static uint32_t _tim2_cc2=0;
static uint32_t _tim2_cc3=0;
static uint32_t _tim2_cc4=0;

//isr for timer1 capture / compare
void TIM2_IRQHandler(void) {
	//oc1 portion
	if ((TIM2->SR & TIM_SR_CC1IF) && (TIM2->DIER & TIM_DIER_CC1IE) {TIM2->SR &=~TIM_SR_CC1IF; if (TIM2->CCER & TIM_CCER_CC1E) _tim2_cc1 = TIM2->CCR1; else TIM2->CCR1 += _tim2_cc1; _tim2_cc1isrptr();}
	if ((TIM2->SR & TIM_SR_CC2IF) && (TIM2->DIER & TIM_DIER_CC2IE) {TIM2->SR &=~TIM_SR_CC2IF; if (TIM2->CCER & TIM_CCER_CC2E) _tim2_cc2 = TIM2->CCR2; else TIM2->CCR2 += _tim2_cc2; _tim2_cc2isrptr();}
	if ((TIM2->SR & TIM_SR_CC3IF) && (TIM2->DIER & TIM_DIER_CC3IE) {TIM2->SR &=~TIM_SR_CC3IF; if (TIM2->CCER & TIM_CCER_CC3E) _tim2_cc3 = TIM2->CCR3; else TIM2->CCR3 += _tim2_cc3; _tim2_cc3isrptr();}
	if ((TIM2->SR & TIM_SR_CC4IF) && (TIM2->DIER & TIM_DIER_CC4IE) {TIM2->SR &=~TIM_SR_CC4IF; if (TIM2->CCER & TIM_CCER_CC4E) _tim2_cc4 = TIM2->CCR4; else TIM2->CCR4 += _tim2_cc4; _tim2_cc4isrptr();}

	//ovf isr
	if ((TIM2->SR & TIM_SR_UIF) && (TIM2->DIER & TIM_DIER_UIE) {TIM2->SR &=~TIM_SR_UIF; _tim2_ovfisrptr();}
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
	TIM2->SR &=~TIM_SR_UIF;				//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_UIE;			//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM2_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC1SetPR(uint32_t pr) {
	//save the period value
	_tim2_cc1 = pr - 0;
	TIM2->CCR1 = TIM2->CNT + _tim2_cc1;

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

//set TIM2_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC2SetPR(uint32_t pr) {
	//save the period value
	_tim2_cc2 = pr - 0;
	TIM2->CCR2 = TIM2->CNT + _tim2_cc2;

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

//set TIM2_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC3SetPR(uint32_t pr) {
	//save the period value
	_tim2_cc3 = pr - 0;
	TIM2->CCR3 = TIM2->CNT + _tim2_cc3;

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

//set TIM2_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr2OC4SetPR(uint32_t pr) {
	//save the period value
	_tim2_cc4 = pr - 0;
	TIM2->CCR4 = TIM2->CNT + _tim2_cc4;

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
//end tmr2
#endif	//TIM2

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

	//ovf isr
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

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC1SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc1 = pr - 0;
	TIM3->CCR1 = TIM3->CNT + _tim3_cc1;

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

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC2SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc2 = pr - 0;
	TIM3->CCR2 = TIM3->CNT + _tim3_cc2;

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

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC3SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc3 = pr - 0;
	TIM3->CCR3 = TIM3->CNT + _tim3_cc3;

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

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3OC4SetPR(uint16_t pr) {
	//save the period value
	_tim3_cc4 = pr - 0;
	TIM3->CCR4 = TIM3->CNT + _tim3_cc4;

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

#if defined(TIM4)
//tmr4
//global variables
static void (* _tim4_ovfisrptr)(void)=empty_handler;				//TIM4_ptr pointing to empty_handler by default
static void (* _tim4_cc1isrptr)(void)=empty_handler;				//TIM4_ptr pointing to empty_handler by default
static void (* _tim4_cc2isrptr)(void)=empty_handler;				//TIM4_ptr pointing to empty_handler by default
static void (* _tim4_cc3isrptr)(void)=empty_handler;				//TIM4_ptr pointing to empty_handler by default
static void (* _tim4_cc4isrptr)(void)=empty_handler;				//TIM4_ptr pointing to empty_handler by default

static uint16_t _tim4_cc1=0;				//output compare registers
static uint16_t _tim4_cc2=0;
static uint16_t _tim4_cc3=0;
static uint16_t _tim4_cc4=0;

//isr for timer1 capture / compare
void TIM4_IRQHandler(void) {
	//oc1 portion
	if ((TIM4->SR & TIM_SR_CC1IF) && (TIM4->DIER & TIM_DIER_CC1IE) {TIM4->SR &=~TIM_SR_CC1IF; if (TIM4->CCER & TIM_CCER_CC1E) _tim4_cc1 = TIM4->CCR1; else TIM4->CCR1 += _tim4_cc1; _tim4_cc1isrptr();}
	if ((TIM4->SR & TIM_SR_CC2IF) && (TIM4->DIER & TIM_DIER_CC2IE) {TIM4->SR &=~TIM_SR_CC2IF; if (TIM4->CCER & TIM_CCER_CC2E) _tim4_cc2 = TIM4->CCR2; else TIM4->CCR2 += _tim4_cc2; _tim4_cc2isrptr();}
	if ((TIM4->SR & TIM_SR_CC3IF) && (TIM4->DIER & TIM_DIER_CC3IE) {TIM4->SR &=~TIM_SR_CC3IF; if (TIM4->CCER & TIM_CCER_CC3E) _tim4_cc3 = TIM4->CCR3; else TIM4->CCR3 += _tim4_cc3; _tim4_cc3isrptr();}
	if ((TIM4->SR & TIM_SR_CC4IF) && (TIM4->DIER & TIM_DIER_CC4IE) {TIM4->SR &=~TIM_SR_CC4IF; if (TIM4->CCER & TIM_CCER_CC4E) _tim4_cc4 = TIM4->CCR4; else TIM4->CCR4 += _tim4_cc4; _tim4_cc4isrptr();}

	//ovf isr
	if ((TIM4->SR & TIM_SR_UIF) && (TIM4->DIER & TIM_DIER_UIE) {TIM4->SR &=~TIM_SR_UIF; _tim4_ovfisrptr();}
}

//initialize the timer1 (16bit)
void tmr4Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM4EN;

	//reset timer
	tmrInit(TIM4, ps);
}

//activate the isr handler
void tmr4OVFAttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);			//disable irq

	_tim4_ovfisrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr4OC1SetPR(uint16_t pr) {
	//save the period value
	_tim4_cc1 = pr - 0;
	TIM4->CCR1 = TIM4->CNT + _tim4_cc1;

	//clear the flag
	//TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM4->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr4OC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);			//disable irq

	_tim4_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr4OC2SetPR(uint16_t pr) {
	//save the period value
	_tim4_cc2 = pr - 0;
	TIM4->CCR2 = TIM4->CNT + _tim4_cc2;

	//clear the flag
	//TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM4->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr4OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);			//disable irq

	_tim4_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr4OC3SetPR(uint16_t pr) {
	//save the period value
	_tim4_cc3 = pr - 0;
	TIM4->CCR3 = TIM4->CNT + _tim4_cc3;

	//clear the flag
	//TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM4->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr4OC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);			//disable irq

	_tim4_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr4OC4SetPR(uint16_t pr) {
	//save the period value
	_tim4_cc4 = pr - 0;
	TIM4->CCR4 = TIM4->CNT + _tim4_cc4;

	//clear the flag
	//TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM4->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr4OC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);			//disable irq

	_tim4_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//ic1
void tmr4IC1Init(uint8_t ps) {
	//set up the capture channel 1
	TIM4->CCMR1 = (TIM4->CCMR1 &~TIM_CCMR1_CC1S) | (TIM_CCMR1_CC1S_0);	//0b01->input capture
	TIM4->CCMR1 = (TIM4->CCMR1 &~TIM_CCMR1_IC1PSC) | ((ps<<TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM4->SR &=~TIM_SR_CC1IF; TIM4->DIER &=~TIM_DIER_CC1IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM4->CCER |= TIM_CCER_CC1E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM4 rmp to MCO (=3)
	//TIM4->OR = (TIM4->OR &~TIM4_OR_TI1_RMP) | (((3) << 0) & TIM4_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM4->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM4CH1toPIN)
	TIM4CH1toPIN();											//configure the input capture pin
#endif	//ch1
}

//activate the isr handler
void tmr4IC1AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);		//disable irq

	_tim4_cc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic2
void tmr4IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM4->CCMR1 = (TIM4->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM4->CCMR1 = (TIM4->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM4->SR &=~TIM_SR_CC2IF; TIM4->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM4->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM4 rmp to MCO (=3)
	//TIM4->OR = (TIM4->OR &~TIM4_OR_TI1_RMP) | (((3) << 0) & TIM4_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM4->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM4CH2toPIN)
	TIM4CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr4IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);		//disable irq

	_tim4_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic3
void tmr4IC3Init(uint8_t ps) {
	//set up the capture channel 1
	TIM4->CCMR2 = (TIM4->CCMR2 &~TIM_CCMR2_CC3S) | (TIM_CCMR2_CC3S_0);	//0b01->input capture
	TIM4->CCMR2 = (TIM4->CCMR2 &~TIM_CCMR2_IC3PSC) | ((ps<<TIM_CCMR2_IC3PSC_Pos) & TIM_CCMR2_IC3PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM4->SR &=~TIM_SR_CC3IF; TIM4->DIER &=~TIM_DIER_CC3IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM4->CCER |= TIM_CCER_CC3E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM4 rmp to MCO (=3)
	//TIM4->OR = (TIM4->OR &~TIM4_OR_TI1_RMP) | (((3) << 0) & TIM4_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM4->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM4CH3toPIN)
	TIM4CH3toPIN();											//configure the input capture pin
#endif	//ch3
}

//activate the isr handler
void tmr4IC3AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);		//disable irq

	_tim4_cc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//ic4
void tmr4IC4Init(uint8_t ps) {
	//set up the capture channel 1
	TIM4->CCMR2 = (TIM4->CCMR2 &~TIM_CCMR2_CC4S) | (TIM_CCMR2_CC4S_0);	//0b01->input capture
	TIM4->CCMR2 = (TIM4->CCMR2 &~TIM_CCMR2_IC4PSC) | ((ps<<TIM_CCMR2_IC4PSC_Pos) & TIM_CCMR2_IC4PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM4->SR &=~TIM_SR_CC4IF; TIM4->DIER &=~TIM_DIER_CC4IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM4->CCER |= TIM_CCER_CC4E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM4 rmp to MCO (=3)
	//TIM4->OR = (TIM4->OR &~TIM4_OR_TI1_RMP) | (((3) << 0) & TIM4_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM4->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM4CH4toPIN)
	TIM4CH4toPIN();											//configure the input capture pin
#endif	//ch4
}

//activate the isr handler
void tmr4IC4AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM4_IRQn);		//disable irq

	_tim4_cc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM4->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM4->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM4_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//tim4

#if defined(TIM6) || 0
//tim17
//global variables
static void (* _tim6_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM6_IRQHandler(void) {
	//oc1..4 isr
	//if (TIM6->SR & TIM_SR_CC1IF) {TIM6->SR &=~TIM_SR_CC1IF; if (TIM6->CCER & TIM_CCER_CC1E) _tim6_cc1 = TIM6->CCR1; else TIM6->CCR1 += _tim17_cc1; _tim17_cc1isrptr();}

	//ovf isr
	if (TIM6->SR & TIM_SR_UIF) {TIM6->SR &=~TIM_SR_UIF; _tim6_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//17-bit prescaler. 32-bit used for compatability
void tmr6Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM6EN;

	//reset timer
	tmrInit(TIM6, ps);
}

//install user handler
void tmr6OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM6_IRQn);		//disable irq

	_tim6_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM6->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM6->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM6_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//tim6


#if defined(TIM7)
//tim7
//global variables
static void (* _tim7_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM7_IRQHandler(void) {
	//oc1..4 isr
	//if (TIM6->SR & TIM_SR_CC1IF) {TIM6->SR &=~TIM_SR_CC1IF; TIM6->CCR1 += _tim17_cc1; _tim17_cc1isrptr();}

	//ovf isr
	if (TIM7->SR & TIM_SR_UIF) {TIM7->SR &=~TIM_SR_UIF; _tim7_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//17-bit prescaler. 32-bit used for compatability
void tmr7Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM7EN;

	//reset timer
	tmrInit(TIM7, ps);
}

//install user handler
void tmr7OVFAttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM7_IRQn);		//disable irq

	_tim7_ovfisrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM7->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM7->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM7_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//tim7

#if defined(TIM14)
//global variables
static void (* _tim14_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim14_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim14_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

static uint16_t _tim14_cc1=0;
static uint16_t _tim14_cc2=0;

//isr for timer14
void TIM14_IRQHandler(void) {
	//oc1..4 isr
	if ((TIM14->SR & TIM_SR_CC1IF) && (TIM14->DIER & TIM_DIER_CC1IE)) {TIM14->SR &=~TIM_SR_CC1IF; if (TIM14->CCER & TIM_CCER_CC1E) _tim14_cc1 = TIM14->CCR1; else TIM14->CCR1 += _tim14_cc1; _tim14_cc1isrptr();}
	if ((TIM14->SR & TIM_SR_CC2IF) && (TIM14->DIER & TIM_DIER_CC2IE)) {TIM14->SR &=~TIM_SR_CC2IF; if (TIM14->CCER & TIM_CCER_CC2E) _tim14_cc2 = TIM14->CCR2; else TIM14->CCR2 += _tim14_cc2; _tim14_cc2isrptr();}

	//ovf isr
	if ((TIM14->SR & TIM_SR_UIF) && (TIM14->DIER & TIM_DIER_UIE)) {TIM14->SR &=~TIM_SR_UIF; _tim14_ovfisrptr();}
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

//set TIM14_cc2 period
//pr is 16-bit. 32-bit used for compatability;
void tmr14OC2SetPR(uint16_t pr) {
	//save the period value
	_tim14_cc2 = pr - 0;
	TIM14->CCR2 = _tim14_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr14OC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);			//disable irq

	_tim14_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_CC2IE;		//enable the isr

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

//ic2
void tmr14IC2Init(uint8_t ps) {
	//set up the capture channel 1
	TIM14->CCMR1 = (TIM14->CCMR1 &~TIM_CCMR1_CC2S) | (TIM_CCMR1_CC2S_0);	//0b01->input capture
	TIM14->CCMR1 = (TIM14->CCMR1 &~TIM_CCMR1_IC2PSC) | ((ps<<TIM_CCMR1_IC2PSC_Pos) & TIM_CCMR1_IC2PSC);	//INPUT prescaler: 0->1:1
	//clear the flag and disable the interrupt
	TIM14->SR &=~TIM_SR_CC2IF; TIM14->DIER &=~TIM_DIER_CC2IE;	//CLEAR_BIT the interrupt flag and disable the interrupt
	TIM14->CCER |= TIM_CCER_CC2E;							//1->ENABLE input capture, 0->disable ic 
	//map TIM14 rmp to MCO (=3)
	//TIM14->OR = (TIM14->OR &~TIM14_OR_TI1_RMP) | (((3) << 0) & TIM14_OR_TI1_RMP);	//0->gpio, 1->rtcclk, 2->hse/32, 3->mco
	//TIM14->EGR |=TIM_EGR_UG;								//force an update

#if defined(TIM14CH2toPIN)
	TIM14CH2toPIN();											//configure the input capture pin
#endif	//ch2
}

//activate the isr handler
void tmr14IC2AttachISR(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);		//disable irq

	_tim14_cc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif	//TIM14

#if defined(TIM15) || 0
//tim15
//global variables
static void (* _tim15_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim15_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim15_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

static uint16_t _tim15_cc1=0;
static uint16_t _tim15_cc2=0;

//isr for timer1 capture / compare
void TIM15_IRQHandler(void) {
	//oc1..4 isr
	if ((TIM15->SR & TIM_SR_CC1IF) && (TIM15->DIER & TIM_DIER_CC1IE) {TIM15->SR &=~TIM_SR_CC1IF; if (TIM15->CCER & TIM_CCER_CC1E) _tim15_cc1 = TIM15->CCR1; else TIM15->CCR1 += _tim15_cc1; _tim15_cc1isrptr();}
	if ((TIM15->SR & TIM_SR_CC2IF) && (TIM15->DIER & TIM_DIER_CC2IE) {TIM15->SR &=~TIM_SR_CC2IF; if (TIM15->CCER & TIM_CCER_CC2E) _tim15_cc2 = TIM15->CCR2; else TIM15->CCR2 += _tim15_cc2; _tim15_cc2isrptr();}

	//ovf isr
	if ((TIM15->SR & TIM_SR_UIF) && (TIM15->DIER & TIM_DIER_UIE) {TIM15->SR &=~TIM_SR_UIF; _tim15_ovfisrptr();}
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr15Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM15EN;

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

//oc1
//set TIM15_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr15OC1SetPR(uint16_t pr) {
	//save the period value
	_tim15_cc1 = pr - 0;
	TIM15->CCR1 = _tim15_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr15OC1AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_cc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//oc1
//set TIM15_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr15OC2SetPR(uint16_t pr) {
	//save the period value
	_tim15_cc2 = pr - 0;
	TIM15->CCR2 = _tim15_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr15OC2AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_cc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
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

#if defined(TIM15CH1toPIN)
	TIM15CH1toPIN();											//configure the input capture pin
#endif	//ch1
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

#endif	//TiM15

//tim16
//global variables
static void (* _tim16_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim16_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim16_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

static uint16_t _tim16_cc1=0;
static uint16_t _tim16_cc2=0;

//isr for timer1 capture / compare
void TIM16_IRQHandler(void) {
	//oc1..4 isr
	if ((TIM16->SR & TIM_SR_CC1IF) && (TIM16->DIER & TIM_DIER_CC1IE)) {TIM16->SR &=~TIM_SR_CC1IF; if (TIM16->CCER & TIM_CCER_CC1E) _tim16_cc1 = TIM16->CCR1; else TIM16->CCR1 += _tim16_cc1; _tim16_cc1isrptr();}
	if ((TIM16->SR & TIM_SR_CC2IF) && (TIM16->DIER & TIM_DIER_CC2IE)) {TIM16->SR &=~TIM_SR_CC2IF; if (TIM16->CCER & TIM_CCER_CC2E) _tim16_cc2 = TIM16->CCR2; else TIM16->CCR2 += _tim16_cc2; _tim16_cc2isrptr();}

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

//oc1
//set TIM16_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr16OC1SetPR(uint16_t pr) {
	//save the period value
	_tim16_cc1 = pr - 0;
	TIM16->CCR1 = _tim16_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr16OC1AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_cc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//oc2
//set TIM16_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr16OC2SetPR(uint16_t pr) {
	//save the period value
	_tim16_cc2 = pr - 0;
	TIM16->CCR2 = _tim16_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr16OC2AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_cc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
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

//return captured value
uint32_t tmr16IC1Get(void) {
	return _tim16_cc1;
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
//end tmr16

//tim17
//global variables
static void (* _tim17_ovfisrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim17_cc1isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default
static void (* _tim17_cc2isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

static uint16_t _tim17_cc1=0;
static uint16_t _tim17_cc2=0;

//isr for timer1 capture / compare
void TIM17_IRQHandler(void) {
	//oc1..4 isr
	if ((TIM17->SR & TIM_SR_CC1IF) && (TIM17->DIER & TIM_DIER_CC1IE)) {TIM17->SR &=~TIM_SR_CC1IF; if (TIM17->CCER & TIM_CCER_CC1E) _tim17_cc1 = TIM17->CCR1; else TIM17->CCR1 += _tim17_cc1; _tim17_cc1isrptr();}
	if ((TIM17->SR & TIM_SR_CC2IF) && (TIM17->DIER & TIM_DIER_CC2IE)) {TIM17->SR &=~TIM_SR_CC2IF; if (TIM17->CCER & TIM_CCER_CC2E) _tim17_cc2 = TIM17->CCR2; else TIM17->CCR2 += _tim17_cc2; _tim17_cc2isrptr();}

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

//oc1
//set TIM17_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr17OC1SetPR(uint16_t pr) {
	//save the period value
	_tim17_cc1 = pr - 0;
	TIM17->CCR1 = _tim17_cc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr17OC1AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_cc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//oc2
//set TIM17_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr17OC2SetPR(uint16_t pr) {
	//save the period value
	_tim17_cc2 = pr - 0;
	TIM17->CCR2 = _tim17_cc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr17OC2AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_cc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
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
//end tmr17

//initialize tim32 as a synchronized 32-bit timer
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

	TIM1->CNT = TIM3->CNT = 0;			//reset the counters
	
	//enable the timers, msw first
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the slave timer
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the master timer
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

//initialize tim32 as a synchronized 32-bit timer
//tim17 as master / lsw (including prescaler) to tim1 / msw of the 32-bit timer
//tim1 as slave / msw of the 32-bit timer
void tmr171Init(uint16_t ps) {
	//initialize tim17 as master
	//enable clock to tim3
	tmr17Init(ps); TIM17->CR1 &=~TIM_CR1_CEN;	//configure tim17 but stop it
	TIM17->CR2 = 0x20;					//MMS = 0b010->tim3 as prescaler
	//initialize tim1 as slave
	tmr1Init(1); TIM1->CR1 &=~TIM_CR1_CEN;	//configure tim2 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM1->SMCR = (TIM1->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x03 << 4) | 				//tab 82: 0b011->//slave tim1 driven by tim17 oc1 - need to be configured
			(0x07 << 0) |				//0b111->external trigger on trgo
			TIM_SMCR_MSM |					//1->enable slave mode
			0x00;

	TIM1->CNT = TIM17->CNT = 0;			//reset the counters
	
	//enable the timers, msw first
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the slave timer
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the master timer
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
	TIM17->ARR = (pr - 1);
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

//initialize tim32 as a synchronized 32-bit timer
//tim1 as master / lsw (including prescaler) to tim3 / msw of the 32-bit timer
//tim3 as slave / msw of the 32-bit timer
void tmr13Init(uint16_t ps) {
	//initialize tim3 as master
	//enable clock to tim1
	tmr1Init(ps); TIM1->CR1 &=~TIM_CR1_CEN;	//configure tim1 but stop it
	TIM1->CR2 = 0x20;					//MMS = 0b010->tim1 as prescaler
	//initialize tim1 as slave
	tmr3Init(1); TIM3->CR1 &=~TIM_CR1_CEN;	//configure tim3 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM3->SMCR = (TIM3->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x00 << 4) | 				//tab 73: 0b000->//slave tim3 driven by tim1
			(0x07 << 0) |				//0b111->external trigger on trgo
			TIM_SMCR_MSM |					//1->enable slave mode
			0x00;

	TIM3->CNT = TIM1->CNT = 0;			//reset the counters
	
	//enable the timers, msw first
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the slave timer
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the master timer
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

//initialize tim143 as a synchronized 32-bit timer
//tim14 as master / lsw (including prescaler) to tim3 / msw of the 32-bit timer
//tim3 as slave / msw of the 32-bit timer
void tmr143Init(uint16_t ps) {
	//initialize tim3 as master
	//enable clock to tim1
	tmr14Init(ps); TIM14->CR1 &=~TIM_CR1_CEN;	//configure tim14 but stop it
	TIM14->CR2 = 0x20;					//MMS = 0b010->tim1 as prescaler
	//initialize tim1 as slave
	tmr3Init(1); TIM3->CR1 &=~TIM_CR1_CEN;	//configure tim3 but stop it
	//source from trgo -> enable slave mode and trigger on trgo
	TIM3->SMCR = (TIM3->SMCR &~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
			(0x03 << 4) | 				//tab 73: 0b011->//slave tim3 driven by tim14
			(0x07 << 0) |				//0b111->external trigger on trgo
			TIM_SMCR_MSM |					//1->enable slave mode
			0x00;

	TIM3->CNT = TIM14->CNT = 0;			//reset the counters
	
	//enable the timers, msw first
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the slave timer
	TIM14->CR1|= TIM_CR1_CEN;			//enable the master timer
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
	TIM14->ARR= (pr - 1);
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

//end Timer

//pwm
//generic pwm setup
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwmInit(TIM_TypeDef *TIMx, uint16_t TxCCP_PS) {
	//source from internal clock -> disable slave mode
	TIMx->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIMx->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIMx->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIMx->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIMx->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIMx->CR1 = 0;						//much easier
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
	TIMx->PSC = TxCCP_PS - 1;					//set the prescaler
	TIMx->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIMx->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIMx->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIMx->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIMx->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	TIMx->EGR = 0xff;					//force an update
	TIMx->BDTR|= TIM_BDTR_MOE;				//enable the main output
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

	pwmInit(TIM1, TxCCP_PS);			//initialize the pwm
	

	//configure CCP1..4 pins
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
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM1->CCR1 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH1toPIN)
	TIM1CH1toPIN();					//route TIM1CH1 to GPIO
#endif
#if defined(TIM1CH1NtoPIN)
	TIM1CH1NtoPIN();					//route TIM1CH1 to GPIO
#endif
#endif

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
#endif
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM1->CCR2 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH2toPIN)
	TIM1CH2toPIN();					//route TIM1CH1 to GPIO
#endif
#if defined(TIM1CH2NtoPIN)
	TIM1CH2NtoPIN();					//route TIM1CH1 to GPIO
#endif
#endif

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
#endif
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM1->CCR3 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH3toPIN)
	TIM1CH3toPIN();					//route TIM1CH1 to GPIO
#endif
#if defined(TIM1CH3NtoPIN)
	TIM1CH3NtoPIN();					//route TIM1CH1 to GPIO
#endif
#endif

#if defined(TIM1CH4toPIN)  || defined(TIM1CH4NtoPIN)
	//configure CCP4
	TIM1->CCMR2 = 	(TIM1->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<< 8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
#if defined(TIM1CH4NtoPIN)
					(1<<14) |			//0->disable CC4N, 1->enable CC4N
#else
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
#endif
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM1->CCR4 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM1CH4toPIN)
	TIM1CH4toPIN();					//route TIM1CH1 to GPIO
#endif
#if defined(TIM1CH4NtoPIN)
	TIM1CH4NtoPIN();					//route TIM1CH1 to GPIO
#endif
#endif

	//TIM1->EGR = 0xff;					//force an update
	//TIM1->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer

}

#if defined(TIM2)
/*
MAPR Bits 7:6 TIM2_REMAP[1:0]: TIM2 remapping
These bits are set and cleared by software. They control the mapping of TIM2 channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN) on the GPIO ports.
00: No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
01: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
10: not used
11: Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm2Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	pwmInit(TIM2, TxCCP_PS);			//initialize the pwm

	//configure CCP1..4 pins
#if defined(TIM2CH1toPIN) || defined(TIM2CH1NtoPIN)
	//configure CCP1
	TIM2->CCMR1 = 	(TIM2->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM2->CCER = 	(TIM2->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
#if defined(TIM2CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM2->CCR1 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM2CH1toPIN)
	TIM2CH1toPIN();					//route TIM2CH1 to GPIO
#endif
#if defined(TIM2CH1NtoPIN)
	TIM2CH1NtoPIN();					//route TIM2CH1 to GPIO
#endif
#endif

#if defined(TIM2CH2toPIN) || defined(TIM2CH2NtoPIN)
	//configure CCP2
	TIM2->CCMR1 = 	(TIM2->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM2->CCER = 	(TIM2->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
#if defined(TIM2CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM2->CCR2 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM2CH2toPIN)
	TIM2CH2toPIN();					//route TIM2CH1 to GPIO
#endif
#if defined(TIM2CH2NtoPIN)
	TIM2CH2NtoPIN();					//route TIM2CH1 to GPIO
#endif
#endif

#if defined(TIM2CH3toPIN) || defined(TIM2CH3NtoPIN)
	//configure CCP3
	TIM2->CCMR2 = 	(TIM2->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM2->CCER = 	(TIM2->CCER &~(0x0f << 8)) |
					(0<<11) |			//0->normal polarity for CC3N, 1->inverse polarity
#if defined(TIM2CH3NtoPIN)
					(1<<10) |			//0->disable CC3N, 1->enable CC3N
#else
					(0<<10) |			//0->disable CC3N, 1->enable CC3N
#endif
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM2->CCR3 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM2CH3toPIN)
	TIM2CH3toPIN();					//route TIM2CH1 to GPIO
#endif
#if defined(TIM2CH3NtoPIN)
	TIM2CH3NtoPIN();					//route TIM2CH1 to GPIO
#endif
#endif

#if defined(TIM2CH4toPIN)  || defined(TIM2CH4NtoPIN)
	//configure CCP4
	TIM2->CCMR2 = 	(TIM2->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<< 8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM2->CCER = 	(TIM2->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
#if defined(TIM2CH4NtoPIN)
					(1<<14) |			//0->disable CC4N, 1->enable CC4N
#else
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
#endif
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM2->CCR4 = 0;						//0% duty cycle

	//configure gpio
#if defined(TIM2CH4toPIN)
	TIM2CH4toPIN();					//route TIM2CH1 to GPIO
#endif
#if defined(TIM2CH4NtoPIN)
	TIM2CH4NtoPIN();					//route TIM2CH1 to GPIO
#endif
#endif

	//TIM2->EGR = 0xff;					//force an update
	//TIM2->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM2->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//tim2

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
#if defined(TIM3CH1NtoPin)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM3->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH1toPIN)
	TIM3CH1toPIN();
#endif
#if defined(TIM3CH1NtoPIN)
	TIM3CH1NtoPIN();
#endif
#endif

#if defined(TIM3CH2toPIN) || defined(TIM3CH2NtoPIN)
	//configure CCP2
	TIM3->CCMR1 = 	(TIM3->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<< 8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
#if defined(TIM3CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM3->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH2toPIN)
	TIM3CH2toPIN();
#endif
#if defined(TIM3CH2NtoPIN)
	TIM3CH2NtoPIN();
#endif
#endif

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
#endif
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM3->CCR3 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH3toPIN)
	TIM3CH3toPIN();
#endif
#if defined(TIM3CH3NtoPIN)
	TIM3CH3NtoPIN();
#endif
#endif

#if defined(TIM3CH4toPIN) || defined(TIM3CH4NtoPIN)
	//configure CCP4
	TIM3->CCMR2 = 	(TIM3->CCMR2 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<< 8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
#if defined(TIM3CH4NtoPIN)
					(1<<14) |			//0->disable CC4N, 1->enable CC4N
#else
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
#endif
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM3->CCR4 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM3CH4toPIN)
	TIM3CH4toPIN();
#endif
#if defined(TIM3CH4NtoPIN)
	TIM3CH4NtoPIN();
#endif
#endif

	//TIM3->EGR = 0xff;					//force an update
	//TIM3->BDTR|= TIM_BDTR_MOE;			//enable the main output
	//enable the timer.
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer

}


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
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM14->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM14CH1toPIN)
	TIM14CH1toPIN();
#endif
#if defined(TIM14CH1NtoPIN)
	TIM14CH1NtoPIN();
#endif
#endif

	//TIM14->EGR = 0xff;					//force an update
	//TIM14->BDTR|= TIM_BDTR_MOE;			//enable the main output
	//enable the timer.
	TIM14->CR1 |= TIM_CR1_CEN;			//enable the timer

}


#if defined(TIM15)
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APBENR2 |= RCC_APBENR2_TIM15EN;

	pwmInit(TIM15, TxCCP_PS);			//initialize the pwm

	//configure CCP1..4
#if defined(TIM15CH1toPIN)  || defined(TIM14CH1NtoPIN)
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
#if defined(TIM14CH1NtoPIN)
					(1<< 2) |			//0->disable CC1N, 1->enable CC1N
#else
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM15->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM15CH1toPIN)
	TIM15CH1toPIN();
#endif
#if defined(TIM15CH1NtoPIN)
	TIM15CH1NtoPIN();
#endif
#endif

#if defined(TIM15CH2toPIN) || defined(TIM15CH2NtoPIN)
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
#if defined(TIM14CH2NtoPIN)
					(1<< 6) |			//0->disable CC2N, 1->enable CC2N
#else
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
#endif
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM15->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM15CH2toPIN)
	TIM15CH2toPIN();
#endif
#if defined(TIM15CH2NtoPIN)
	TIM15CH2NtoPIN();
#endif
#endif

	//TIM15->EGR = 0xff;					//force an update
	//TIM15->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM15->CR1 |= TIM_CR1_CEN;			//enable the timer

}
#endif	//TIM15

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
#if defined(TIM16CH1toPIN) || defined(T16CH1NtoPIN)
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
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM16->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM16CH1toPIN)
	TIM16CH1toPIN();
#endif
#if defined(TIM16CH1NtoPIN)
	TIM16CH1NtoPIN();
#endif

#endif

	//TIM16->EGR = 0xff;					//force an update
	//TIM16->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM16->CR1 |= TIM_CR1_CEN;			//enable the timer

}

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
#endif
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM17->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
#if defined(TIM17CH1toPIN)
	TIM17CH1toPIN();
#endif
#if defined(TIM17CH1NtoPIN)
	TIM17CH1NtoPIN();
#endif
#endif
	//TIM17->EGR = 0xff;					//force an update
	//TIM17->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer

}
//end pwm

#if defined(LPTIM1) || defined(LPTIM2)
//lptim1 and lptim2
//generic initialization code for lptim
//initialize the timer1 (16bit)
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
#endif	//lptim1 and lptim2

#if defined(LPTIM1)
//tmr1
//global variables
static void (* lptim1_isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc1isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc2isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc3isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc4isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default

static uint16_t lptim1_pr=0;				//output compare registers
//static uint16_t lptim1_cc2=0;
//static uint16_t lptim1_cc3=0;
//static uint16_t lptim1_cc4=0;

//isr for timer1 capture / compare
void LPTIM1_IRQHandler(void) {
	LPTIM1->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	//may want to process this in the user isr _lptim1_isrptr()
	LPTIM1->CR |= LPTIM_CR_SNGSTRT;				//1->start the timer, 0->stop the timer
	lptim1_isrptr();							//execute user isr
}

//initialize the timer1 (16bit)
void lptmr1Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_LPTIM1EN;

	lptmrInit(LPTIM1, ps);					//initialize the timer
	LPTIM1->CR &=~LPTIM_CR_CNTSTRT;			//1->start the counter, 0->don't start the timer yet
}

//activate the isr handler
void lptmr1AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(LPTIM1_IRQn);		//disable irq

	lptim1_isrptr = isr_ptr;			//install user handler

	//clear the flag
	LPTIM1->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	LPTIM1->IER|= LPTIM_IER_ARRMIE;				//0->disable the interrupt

	NVIC_EnableIRQ(LPTIM1_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif	//lptim1

#if defined(LPTIM2)
//tmr2
//global variables
static void (* lptim2_isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc1isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc2isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc3isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default
//static void (* lptim1_cc4isrptr)(void)=empty_handler;				//lptim1_ptr pointing to empty_handler by default

static uint16_t lptim2_pr=0;				//output compare registers
//static uint16_t lptim1_cc2=0;
//static uint16_t lptim1_cc3=0;
//static uint16_t lptim1_cc4=0;

//isr for timer1 capture / compare
void LPTIM2_IRQHandler(void) {
	LPTIM2->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	//may want to process this in the user isr _lptim1_isrptr()
	LPTIM2->CR |= LPTIM_CR_SNGSTRT;				//1->start the timer, 0->stop the timer
	lptim2_isrptr();							//execute user isr
}

//initialize the timer1 (16bit)
void lptmr2Init(uint16_t ps) {
	//route the clock to timer
	RCC->APBENR1 |= RCC_APBENR1_LPTIM2EN;

	lptmrInit(LPTIM2, ps);					//initialize the timer
	LPTIM2->CR &=~LPTIM_CR_CNTSTRT;			//1->start the counter, 0->don't start the timer yet
}

//activate the isr handler
void lptmr2AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(LPTIM2_IRQn);		//disable irq

	lptim2_isrptr = isr_ptr;			//install user handler

	//clear the flag
	LPTIM2->ICR = LPTIM_ICR_ARRMCF;				//write 1 to clear the flag
	LPTIM2->IER|= LPTIM_IER_ARRMIE;				//0->disable the interrupt

	NVIC_EnableIRQ(LPTIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif	//lptim2
//end lptim1 and lptim2

//adc module
//global variables
static uint32_t _adc_calfactor;				//adc calibration factor
//initialize adc
void adc1Init(void) {
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;		//enable clock to ADC1

	//configure adc
	ADC1->CFGR1 = 	(0x00<<26) |			//0->awd on channel 0
					(0<<23) |				//0->AWD disabled
					(0<<22) |				//0->AWD on all channels (but disabled by bit 22
					(1<<16) |				//1->enable discontinuous conversion
					(0<<15) |				//0->auto off disabled
					(0<<14) |				//0->wait conversion disabled
					(0<<13) |				//0->single conversion mode, 1->continuous mode
					(0<<12) |				//0->adc data register preserved in overrun
					(0x00<<10) |			//00->hardware external trigger disabled
					(0x00<<6) |				//0000->external on TRG0 - but disabled
					(0<<5) |				//0->right aligned, 1->left aligned
					(0x00<<3) |				//00->data resolution = 12bit, 01->10bit, 10->8bit, 11->6bit
					(0<<2) |				//0->upward scan
					(0<<1) |				//0->DMA one shot mode selected
					(0<<0) |				//0->DMA disabled
					0x00;
	ADC1->CFGR2 = 	(0x02ul << 30);			//00->adc clock, 01->PCLK/2, 10->PCLK/4 -> no jitter
	//set adc sample time
	//0b111->239.5 cycles for all channels
	ADC1->SMPR = 	(ADC_SMPR_SMP1 << (3 * 0)) |
					0x00;
	//set adc channel sequence
	//ADC1->SQR3 = ADC1->SQR2 = ADC1->SQR1 = 0;							//0->1 conversion

	//start self-calibration
	ADC1->CR =	0;							//reset CR
	ADC1->CR = (1<<15);						//start the calibration
	while (ADC1->CR & (1<<15)) continue;	//wait for ADC calibration to finish
	_adc_calfactor = ADC1->DR;				//save adc calibration factor

	//optional: enable temperature sensors
	ADC->CCR |= 	(1ul<<23) |				//1->enable temperature sensor
					(1ul<<22) |				//1->enable Vrefint. 1.20v nominal
					0x00;

	ADC1->CR = 	(1<<0);						//enable aden
	while ((ADC1->ISR & (1<<0)) == 0) continue;	//wait for the adc to be ready
}

//analog to digital converter on ADC1
//ain/analog input channel: ain valid values: 0..15, 16=temperature sensor, 17=Vrefint
//***user needs to be configured as floating input***
uint16_t adc1Read(uint32_t ain) {
	//ADC1->ISR &= ~(1<<2);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);					//start the conversion
	ADC1->CHSELR = ain & 0x03fffful;				//define the first (and only) adc ch
	ADC1->CR |= (1<<2);						//start conversion
	while ((ADC1->ISR & (1<<2)) == 0) continue;	//wait for conversion to end (EOC set at end of conversion)
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
#endif	//0


//spi
//generic reset. br=0..7
void spiInit(SPI_TypeDef *SPIx, uint32_t br) {
	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	//master only, clock polarity: 0 at idle, clock phase: capture on first edge
	SPIx->CR1 = SPIx->CR2 = SPIx->SR = 0;	//reset the spi module
	SPIx->CR1 = (SPIx->CR1 &~SPI_CR1_BR) | ((br << SPI_CR1_BR_Pos) & SPI_CR1_BR);
	SPIx->CR1 |= SPI_CR1_MSTR;			//1->master mode, 0->slave mode
	SPIx->CR2 = (SPIx->CR2 &~SPI_CR2_DS) | ((0b0111<< SPI_CR2_DS_Pos) & SPI_CR2_DS);	//8bit data length: default
	SPIx->CR2 &=~SPI_CR2_TXEIE;			//0->disable tx empty interrupt
	SPIx->CR2 &=~SPI_CR2_RXNEIE;		//0->disable rx not empty  interrupt
	SPIx->CR2 &=~SPI_CR2_ERRIE;			//0->disable error interrupt
	SPIx->CR2 &=~SPI_CR2_FRF;			//0->motorolla frame mode, 1->TI frame mode
	SPIx->CR2 &=~SPI_CR2_NSSP;			//0->no nss pin mgmt
	SPIx->CR2 &=~SPI_CR2_SSOE;			//0->SS output disabled
	//IFS0bits.SPIxIF = 0;				//0->reset the flag
	//IEC0bits.SPIxIE = 0;				//0->disable the interrupt, 1->enable the interrupt
	SPIx->CR1 |= SPI_CR1_SPE;			//1->enable the module, 0->disable the module
}

//rest spi1
void spi1Init(uint32_t br) {
	uint32_t tmp;
	RCC->APBENR2 |= RCC_APBENR2_SPI1EN;	//1->enable the module
	
	//initialize the spi module
	spiInit(SPI1, br);
	
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
	spiInit(SPI2, br);
	
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
//i2c
//i2c1 initialization
void i2cInit(I2C_TypeDef *I2Cx, uint32_t br) {
	//configure i2c
	I2Cx->CR1 = I2Cx->CR2 = I2Cx->ISR = 0;		//initialize the registers
	I2Cx->ICR = 0xffff;					//write 1 to clear
	//need to figure out timing parameters
	//simple calibration of i2c timing -> may need refinement by user
	I2Cx->TIMINGR = (I2Cx->TIMINGR &~I2C_TIMINGR_SCLDEL)| ((4  << 20) & I2C_TIMINGR_SCLDEL);	//set scldel
	I2Cx->TIMINGR = (I2Cx->TIMINGR &~I2C_TIMINGR_SDADEL)| ((4  << 16) & I2C_TIMINGR_SDADEL);	//set sdadel
	I2Cx->TIMINGR = (I2Cx->TIMINGR &~I2C_TIMINGR_SCLH  )| ((16 <<  8) & I2C_TIMINGR_SCLH  );	//set sclh
	I2Cx->TIMINGR = (I2Cx->TIMINGR &~I2C_TIMINGR_SCLL  )| ((16 <<  0) & I2C_TIMINGR_SCLL  );	//set scll
	br = ((I2Cx==I2C1)?F_I2C1:F_I2C2) / br / (I2Cx->TIMINGR & I2C_TIMINGR_SCLL) / 2 + 1;								//br is now the baud rate: br is at least 1
	I2Cx->TIMINGR = (I2Cx->TIMINGR &~I2C_TIMINGR_PRESC )| ((br << 28) & I2C_TIMINGR_PRESC );	//set the baud rate
	
	//enable the i2c
	I2Cx->CR1 |= I2C_CR1_PE;			//1->enable the i2c module
	
}

//i2c1 initialization
void i2c1Init(uint32_t br) {
	RCC->APBENR1 |= RCC_APBENR1_I2C1EN;	//1->enable clock

	//configure i2c
	i2cInit(I2C1, br);
	
//#if defined(SCL1toPIN)
	SCL1toPIN();						//enable scl pin - you must have SCL pins
//#endif

//#if defined(SDA1toPIN)
	SDA1toPIN();						//enable sda pin - you mus thave SDA pins
//#endif
}

#if defined(I2C2)
//i2c2 initialization
//i2c
//i2c1 initialization
void i2c2Init(uint32_t br) {
	RCC->APBENR1 |= RCC_APBENR1_I2C1EN;	//1->enable clock

	//configure i2c
	i2cInit(I2C2, br);

//#if defined(SCL1toPIN)
	SCL1toPIN();						//enable scl pin - you must have SCL pins
//#endif

//#if defined(SDA1toPIN)
	SDA1toPIN();						//enable sda pin - you mus thave SDA pins
//#endif
}

#endif	//i2c2
//end i2c

//rtcc
//initialize the rtc
//prep: see RTC_PREP in the datasheet
//rtc_clk: RCC_BDCR_RTCSEL_NOCLOCK, RCC_BDCR_RTCSEL_LSI, RCC_BDCR_RTCSEL_LSE and RCC_BDCR_RTCSEL_HSE
//prep consists of a 7-bit prediv_a (msb) and 15-bit prediv_b (lsb)
void RTCInit(uint32_t rtc_clk, uint32_t prep) {
	uint16_t prediv_a, prediv_s;
/*----------------------------------------------------------------------------
 STM32 Real Time Clock setup.
 initializes the RTC Prescaler and RTC counter register
 *----------------------------------------------------------------------------*/
//__inline static void stm32_RtcSetup (void) {

	RCC->APBENR1 |= RCC_APBENR1_PWREN | RCC_APBENR1_RTCAPBEN;		// enable clock for Power interface
	PWR->CR1 |= PWR_CR1_DBP;										// enable access to RTC, BDC registers

	switch (rtc_clk & RCC_BDCR_RTCSEL) {
		case RCC_BDCR_RTCSEL_NOCLOCK: return; break;											//no clock
		case RCC_BDCR_RTCSEL_LSE: RCC->BDCR |= RCC_BDCR_LSEON; while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0) continue; break;							//lse
		case RCC_BDCR_RTCSEL_HSE: RCC->CR   |= RCC_CR_HSEON;   while ((RCC->CR  & RCC_CR_HSERDY ) == 0  ) continue; break;							//hse / 32
		default: RCC->CSR  |= RCC_CSR_LSION;  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0  ) continue; break;											//LSI
	}

	RCC->BDCR &=~(RCC_BDCR_RTCSEL | RCC_BDCR_RTCEN);
	RCC->BDCR |= (rtc_clk & RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN;		// set RTC clock source, enable RTC clock

	//RCC->BDCR |= (rtc_clk | RCC_BDCR_RTCEN);						// set RTC clock source, enable RTC clock

//	RTC->CRL	 &= ~(1<<3);										// reset Registers Synchronized Flag
//	while ((RTC->CRL & (1<<3)) == 0);								// wait until registers are synchronized

	//disable write protection by writing 0xca, 0x53 to WPR
	RTC->WPR = 0xca; RTC->WPR = 0x53;

	//enter initialization mode
	RTC->ICSR |= RTC_ICSR_INIT;										//set the init bit to enter the initialization mode
	while ((RTC->ICSR & RTC_ICSR_INITF) == 0) continue;				//wait for INITF to go to 1 -> entering the init mode
	//set up the RTC: time, prescaler, ...
	//set prescaler
	prep = prep - 1;												//minus 1
	prep = prep &~(1<<0);											//zero the last digit - to ensure it is an even number
	prediv_a=1; prediv_s=prep;
	while ((prediv_s & 0x01) == 0) {
		prediv_s = prediv_s / 2;
		prediv_a = prediv_a * 2;
		if (prediv_a == 0x7f) break;								//prediv_a max is 0x7f
		if (prediv_a > 0x7fff) prediv_a = prediv_a &~(1<<0);		//prediv_s must be less than 32768
	}
	prediv_a = prediv_a-1; prediv_s = prediv_s-1;
	//RTC->PRER = (prep & 0x007f7ffful);							//set prescaler = prev-a and prev-s
	RTC->PRER = (prediv_a << 16) | prediv_s;

	//set the TR/DR registers

	//end set the TR/DR registers

	//done setting upt he RTC
	//exit the init mode
	RTC->ICSR &=~RTC_ICSR_INIT;										//clear the init bit to exit init mode

	//writing anything else to WPR reactivates the write protection
	RTC->WPR = 0x00;

#if 0
	RTC->CRL	|=	RTC_CRL_CNF;																		// set configuration mode
	RTC->PRLH	= ((__RTC_PERIOD*__RTCCLK/1000-1)>>16) & 0x00FF;	 // set prescaler load register high
	RTC->PRLL	= ((__RTC_PERIOD*__RTCCLK/1000-1)		) & 0xFFFF;	 // set prescaler load register low
	RTC->CNTH	= ((__RTC_CNT)>>16) & 0xFFFF;											// set counter high
	RTC->CNTL	= ((__RTC_CNT)		) & 0xFFFF;											// set counter low
	RTC->ALRH	= ((__RTC_ALR)>>16) & 0xFFFF;											// set alarm high
	RTC->ALRL	= ((__RTC_ALR)		) & 0xFFFF;											// set alarm low
	if (__RTC_INTERRUPTS) {											// RTC interrupts used
		RTC->CRH = __RTC_CRH;										// enable RTC interrupts
		NVIC->ISER[0] |= (1 << (RTC_IRQChannel & 0x1F));			// enable interrupt
	}
	RTC->CRL	&= ~RTC_CRL_CNF;									// reset configuration mode
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0);						// wait until write is finished
#endif

	PWR->CR1	 &= ~PWR_CR1_DBP;									// disable access to RTC registers
} // end of stm32_RtcSetup


//set rtc time
void RTCSetTime(uint32_t val) {
	//change dr/tr registers
	//RCC->APB1ENR |= RCC_APB1ENR_PWREN;							// enable clock for Power interface
	PWR->CR1			|= PWR_CR1_DBP;								// enable access to RTC, BDC registers

	//disable write protection by writing 0xca, 0x53 to WPR
	RTC->WPR = 0xca; RTC->WPR = 0x53;

	//enter initialization mode
	RTC->ICSR |= RTC_ICSR_INIT;										//set the init bit to enter the initialization mode
	while ((RTC->ICSR & RTC_ICSR_INITF) == 0) continue;				//wait for INITF to go to 1 -> entering the init mode
	//set up the RTC: time, prescaler, ...
	//set dr
	//RTC->DR = tmp_dr;
	//set TR
	RTC->TR = val;

	//end set the TR/DR registers

	//done setting upt he RTC
	//exit the init mode
	RTC->ICSR &=~RTC_ICSR_INIT;										//clear the init bit to exit init mode

	//writing anything else to WPR reactivates the write protection
	RTC->WPR = 0x00;
	PWR->CR1	 &= ~PWR_CR1_DBP;									// disable access to RTC registers
}

//set rtc time
void RTCSetDate(uint32_t val) {
	//change dr/tr registers
	//RCC->APB1ENR |= RCC_APB1ENR_PWREN;														// enable clock for Power interface
	PWR->CR1			|= PWR_CR1_DBP;																	 // enable access to RTC, BDC registers

	//RTC->ISR &=~RTC_ISR_RSF; while ((RTC->ISR & RTC_ISR_RSF) == 0) continue;	//wait for calendar to be synchronized
	//disable write protection by writing 0xca, 0x53 to WPR
	RTC->WPR = 0xca; RTC->WPR = 0x53;

	//enter initialization mode
	RTC->ICSR |= RTC_ICSR_INIT;										//set the init bit to enter the initialization mode
	while ((RTC->ICSR & RTC_ICSR_INITF) == 0) continue;				//wait for INITF to go to 1 -> entering the init mode
	//set up the RTC: time, prescaler, ...
	//set dr
	RTC->DR = val;
	//set TR
	//RTC->TR = val;

	//end set the TR/DR registers

	//done setting upt he RTC
	//exit the init mode
	RTC->ICSR &=~RTC_ICSR_INIT;										//clear the init bit to exit init mode

	//writing anything else to WPR reactivates the write protection
	RTC->WPR = 0x00;
	PWR->CR1	 &= ~PWR_CR1_DBP;																		 // disable access to RTC registers
}


//convert rtcc to time_t
time_t RTC2time(time_t *t) {
	struct tm tm;
	uint32_t tmp;

	//read the rtcc
#if 0
	tmp = RTCGetTime();
	tm.tm_sec = BCD2DEC((tmp >> 0) & 0x7f);	
	tm.tm_min = BCD2DEC((tmp >> 8) & 0x7f);
	tm.tm_hour= BCD2DEC((tmp >>16) & 0x3f);
	tmp = RTCGetDate();
	tm.tm_mday= BCD2DEC((tmp >> 0) & 0x3f); 
	tm.tm_mon = BCD2DEC((tmp >> 8) & 0x1f) - 1;		//month = 0..11
	tm.tm_wday= BCD2DEC((tmp >>13) & 0x07);		//RTCGetWDay()-1;				//1..7
	tm.tm_year= BCD2DEC((tmp >>16) & 0xff)+2000-1900;		//RTCGetYear()+2000-1900;	//dspic33 RTCC starts at 1/1/2000, posix year relative to 1/1/1900
	tm.tm_isdst=0;						//0->no savings time
#else
	tm.tm_sec = RTCGetSec();
	tm.tm_min = RTCGetMin();
	tm.tm_hour= RTCGetHour();
	tm.tm_mday= RTCGetDay();
	//tm.tm_wday= RTCGetWDay()-1;			//1..7
	tm.tm_mon = RTCGetMonth()-1;		//month = 0..11
	tm.tm_year= RTCGetYear()+2000-1900;	//dspic33 RTCC starts at 1/1/2000, posix year relative to 1/1/1900
	tm.tm_isdst=0;						//0->no savings time
#endif
	if (t==NULL) return mktime(&tm); 						//just return seconds since epoch time if pointer is NULL
	return *t=mktime(&tm);									//change pointed value + return seconds since epoch time
}

//turn time (=time_t) to RTCC
time_t time2RTC(time_t t) {
	struct tm *tmptr;
	//uint16_t year, mondat, wkhour, minsec;
	uint32_t tmp;

	t=(t<946684800ul)?946684800ul:t;					//minimum time is 1/1/2000 for the hardware rtc
	tmptr=gmtime(&t);
	tmp =              DEC2BCD(tmptr->tm_hour);
	tmp = (tmp << 8) | DEC2BCD(tmptr->tm_min);
	tmp = (tmp << 8) | DEC2BCD(tmptr->tm_sec);
	RTCSetTime(tmp);
	tmp =              DEC2BCD(tmptr->tm_year+1900-2000);
	tmp = (tmp << 8) | DEC2BCD(tmptr->tm_mon+1);
	tmp = (tmp << 8) | DEC2BCD(tmptr->tm_mday);
	RTCSetDate(tmp);
	return t;
}

//set calibration
//only 16-sec is supported
int16_t RTCSetCal(int16_t cal) {
	PWR->CR1			|= PWR_CR1_DBP;																	 // enable access to RTC, BDC registers
	//disable write protection by writing 0xca, 0x53 to WPR
	RTC->WPR = 0xca; RTC->WPR = 0x53;
	//bound cal to -511..511;
	RTC->CALR = RTC->CALR &~(RTC_CALR_CALW8 | RTC_CALR_CALW16);		//clear bit 14 (CALW8=0) and set bit 13 (CALW16=1)
	if (cal > 0) {RTC->CALR |= RTC_CALR_CALP;}				//adding pulses -> set bit 16 (CALP=1)
	else {RTC->CALR &=~RTC_CALR_CALP; cal = -cal;} 		//subtracting pulses -> clear bit 15 (CALP=0)
	RTC->CALR = (RTC->CALR &~0x1f) | (cal & 0x1f);	//change the CALM bits
	//writing anything else to WPR reactivates the write protection
	RTC->WPR = 0x00;
	PWR->CR1			&=~PWR_CR1_DBP;																	 // disable access to RTC, BDC registers
	return RTC->CALR & 0x1f;
}

//set calibration
int16_t RTCGetCal(void) {
	if (RTC->CALR & RTC_CALR_CALP) return 512 - (RTC->CALR & 0x1ff);
	else return -(RTC->CALR & 0x1ff);
}

//test for subsecond: 0->first half of a second, 1->second half of a second
uint8_t RTCHalfsec(void) {
	return (RTC->SSR < (RTC->PRER & 0x7fff) / 2)?0:1;
}
//end rtcc

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


//cnint
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
	//if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 0)) {EXTI->FPR1 |= 1<< 0; EXTI->RPR1 |= 1<< 0; _exti0_isrptr();}
	//if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 1)) {EXTI->FPR1 |= 1<< 1; EXTI->RPR1 |= 1<< 1; _exti1_isrptr();}
	if (SYSCFG->IT_LINE_SR[5] & (1<< 0)) {SYSCFG->IT_LINE_SR[5] &=~(1<< 0); _exti0_isrptr();}
	if (SYSCFG->IT_LINE_SR[5] & (1<< 1)) {SYSCFG->IT_LINE_SR[5] &=~(1<< 1); _exti1_isrptr();}
}

//EXTI2-3 ISR
void EXTI2_3_IRQHandler(void) {
	//clear the flag and execute user isr
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 2)) {EXTI->FPR1 |= 1<< 2; EXTI->RPR1 |= 1<< 2; _exti2_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 3)) {EXTI->FPR1 |= 1<< 3; EXTI->RPR1 |= 1<< 3; _exti3_isrptr();}
}

//EXTI15-10 ISR
void EXTI4_15_IRQHandler(void) {
	//clear the flag and execute user isr
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 4)) {EXTI->FPR1 |= 1<< 4; EXTI->RPR1 |= 1<< 4; _exti4_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 5)) {EXTI->FPR1 |= 1<< 5; EXTI->RPR1 |= 1<< 5; _exti5_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 6)) {EXTI->FPR1 |= 1<< 6; EXTI->RPR1 |= 1<< 6; _exti6_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 7)) {EXTI->FPR1 |= 1<< 7; EXTI->RPR1 |= 1<< 7; _exti7_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 8)) {EXTI->FPR1 |= 1<< 8; EXTI->RPR1 |= 1<< 8; _exti8_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<< 9)) {EXTI->FPR1 |= 1<< 9; EXTI->RPR1 |= 1<< 9; _exti9_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<10)) {EXTI->FPR1 |= 1<<10; EXTI->RPR1 |= 1<<10; _exti10_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<11)) {EXTI->FPR1 |= 1<<11; EXTI->RPR1 |= 1<<11; _exti11_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<12)) {EXTI->FPR1 |= 1<<12; EXTI->RPR1 |= 1<<12; _exti12_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<13)) {EXTI->FPR1 |= 1<<13; EXTI->RPR1 |= 1<<13; _exti13_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<14)) {EXTI->FPR1 |= 1<<14; EXTI->RPR1 |= 1<<14; _exti14_isrptr();}
	if ((EXTI->FPR1 | EXTI->RPR1) & (1<<15)) {EXTI->FPR1 |= 1<<15; EXTI->RPR1 |= 1<<15; _exti15_isrptr();}
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
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
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
		case EXTI_FALLING: 	EXTI->FTSR1 |= pin; EXTI->RTSR1 &=~pin; break;
		case EXTI_RISING: 	EXTI->FTSR1 &=~pin; EXTI->RTSR1 |= pin; break;
		case EXTI_BOTH: 	EXTI->FTSR1 |= pin; EXTI->RTSR1 |= pin; break;
	}
	//disable the interrupts
	EXTI->EXTICR[0] = EXTI->EXTICR[1]=EXTI->EXTICR[2]=EXTI->EXTICR[3]=0x00;
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
	EXTI->RPR1 |= pin;							//1->clear the flag
	EXTI->FPR1 |= pin;							//1->clear the flag
	//enable the interrupt
	EXTI->IMR1 |= pin;							//1->enable the interrupt, 0->disable the interrupt
	EXTI->EMR1 |= pin;							//1->enable the event trigger, 0->disable the event trigger

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
		case (1<<12): EXTI->EXTICR[3] |= gpiox << 0; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti12_isrptr = isr_ptr; break;
		case (1<<13): EXTI->EXTICR[3] |= gpiox << 8; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti13_isrptr = isr_ptr; break;
		case (1<<14): EXTI->EXTICR[3] |= gpiox <<16; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti14_isrptr = isr_ptr; break;
		case (1<<15): EXTI->EXTICR[3] |= gpiox <<24; NVIC_EnableIRQ(EXTI4_15_IRQn); _exti15_isrptr = isr_ptr; break;
	}
}

//end cnint

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

#if defined(RNG)
//true rng
//initialize the rng
void rngInit(void) {
	RCC->AHBENR |= RCC_AHBENR_RNGEN;	//ENABLE rng
	
	//configure the clock
	RCC->CCIPR = (RCC->CCIPR &~RCC_CCIPR_RNGSEL) | (1<<RCC_CCIPR_RNGSEL_Pos); //select the clock. 0->no clock, 1->HSI16, 2->sysclk, 3->pllclk
	RCC->CCIPR = (RCC->CCIPR &~RCC_CCIPR_RNGDIV) | (0<<RCC_CCIPR_RNGDIV_Pos); //set the divider. 0->1:1, 1->2:1, 2->4:1, 3->8:1
	RNG->CR = 0;						//default value
	RNG->SR = 0;						//default value
	RNG->CR|= RNG_CR_RNGEN;				//ENABLE rng
}
//end true rng
#endif	//rng

#if defined(AES)
//AES
//initalize the AES: ECB mode, polling only
//128 bit blocks
void aesInit(void) {
	RCC->AHBENR |= RCC_AHBENR_AESEN;	//1->enable clock, 0->disable clock
	
	AES->CR = 0;						//reset state. AES is disabled
	
	//AES->CR|= AES_CR_EN;				//1->enable aes, 0->disable aes
}

//ciper a message
char *aesCipher(char *message) {
	char *ptr0=message;					//message
	char *ptr1;
	
	AES->CR&=~AES_CR_EN;				//1->enable aes, 0->disable aes
	while (*ptr0) {
		ptr1=ptr0;
		AES->DINR = *ptr0++;
		AES->DINR = *ptr0++;
		AES->DINR = *ptr0++;
		AES->DINR = *ptr0++;
		
		AES->CR|= AES_CR_EN;			//1->enable aes, 0->disable aes
		
		//read out the encrypted data
		*ptr1++ = AES->DOUTR;			//output data
		*ptr1++ = AES->DOUTR;			//output data
		*ptr1++ = AES->DOUTR;			//output data
		*ptr1++ = AES->DOUTR;			//output data
	}
	AES->CR&=~AES_CR_EN;				//1->enable aes, 0->disable aes
	return message;
}

//end AES
#endif	//aes

//tamper backup registers
//reset the tamper to use the backup registers
void bkpInit(void) {
	//enable the clock
	RCC->APBENR1 |= RCC_APBENR1_PWREN | RCC_APBENR1_RTCAPBEN;			//1->enable clock to pwr, 0->disable clock to pwr

	//set dbp bit in pwr
	bkpUnlock();								//1->enable write assess, 0->disable write access
	RCC->BDCR |= RCC_BDCR_RTCEN;				//enable RTC clock
	bkpLock();
}
