#include <inttypes.h>
#include <stdbool.h>

struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
  //structure that holds all the RCC registers that are responsible for managing the clock and reset functionalities of peripherals
  //with some of these registers we are able to turn on peripherals that are turned off initially

};



#define RCC ((struct rcc*) 0x40023800)

struct gpio{
	volatile uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2];
};

#define GPIOD ((struct gpio *) 0x40020C00)
#define GPIOA ((struct gpio *) 0x40020000)

struct systick{
	volatile uint32_t CTRL,LOAD,VAL,CALIB;
};

#define SYSTICK ((struct systick*) 0xE000E010)

struct nvic{
	volatile uint32_t ISER[8U],RESERVED0[24U],ICER[8U], RESERVED1[24U],  ISPR[8U], RESERVED2[24U],
	ICPR[8U], RESERVED3[24U], IABR[8U], RESERVED4[56U], IP[240U], RESERVED5[644U], STIR;
};

#define NVIC ((struct nvic *) 0xE000E100)

struct exti{
	volatile uint32_t IMR,EMR,RTSR,FTSR,SWIER,PR;
};

#define EXTI ((struct exti *) 0x40013C00)

struct syscfg{
	volatile uint32_t MEMRMP,PMC,EXTICR[4],RESERVED[2], CMPCR;
};

#define SYSCFG ((struct syscfg *) 0x40013800)


void systick_init(uint32_t ticks) {
	if(ticks-1>0xffffff ) return;
	SYSTICK->LOAD = ticks-1;
	SYSTICK->VAL =0;
	SYSTICK->CTRL |= (1UL<<0) | (1UL<<1) | (1UL<<2);

}

volatile uint32_t counter;

void SysTick_Handler(void) {
	counter++;
}

void delay(uint32_t time) {
	counter =0;
	while(counter < time) {
		__asm__("nop");
	}
}


void exti_init(void){
	//button is on GPIOA pin 0

	SYSCFG->EXTICR[0] |= (0U <<0); //manual 141
	EXTI->IMR |=(1U<<0);
	EXTI->RTSR &=~(1U<<0);
	EXTI->FTSR |=(1U<<0);

	NVIC->IP[6] = (uint8_t)((3 << (8U - 4U)) & (uint32_t) 0xFFUL); //this is in the file: core_cm4.h under function:
	//__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)

	NVIC->ISER[6 >> 5UL] = (uint32_t)(1UL << (((uint32_t)6)&0x1FUL)); //this is in the file: core_cm4.h under function:
	//__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)

}

volatile uint8_t status;

void EXTI0_IRQHandler(void){
	// if already red delay
	// if green, small delay then change to yellow then red
	// if yellow change to red
	// if red yellow, turn to green then delay for few seconds then switch to normal: yellow, red
	if(EXTI->PR & (1U<<0)) {

		EXTI->PR |=(1U<<0);


		if(GPIOD->ODR & (1U<<12)) {
			//in the timer version try a delay here
			//a delay in this version doesnt work because systick is also na interrupt

			for(int i=0; i < 200000; i++){}
			status = 1;


		}
	}

}
// 1 green
// 2 yellow
// 3 red
// 4 red yellow

#define FREQ 16000000

int main(void) {
		RCC->AHB1ENR |= (1U << 3);
		RCC->AHB1ENR |= (1U << 0); //enable GPIOA
		RCC->APB2ENR |= (1UL << 14);


		GPIOD->MODER |= (1U << 26); //orange
		GPIOD->MODER |= (1U << 24); //green
		GPIOD->MODER |= (1U << 28); //red

		systick_init(FREQ/1000);

		exti_init();
		status = 0;
		while(1){


			//make the lights have a fade in effect somehow

			switch(status){
			case(0): //green light
					GPIOD->ODR |=(1U<<12);
					for(int i =0; i< 5000 && status == 0; i++) {
						delay(1);
					}

					GPIOD->ODR &= ~(1U<<12);

					if(status == 0) status = 1;
					break;
			case(1): //orange
					GPIOD->ODR &= ~(1U<<12);
					GPIOD->ODR |= (1U<<13);
					delay(2000);
					GPIOD->ODR &= ~(1U<<13);
					status = 2;
					break;
			case(2): //red
					GPIOD->ODR |= (1U << 14);
					delay(7000);

					status =3;
					break;
			case(3): //red yellow
					GPIOD->ODR |= (1U << 13);
					delay(1500);
					GPIOD->ODR &= ~(1U << 13);
					GPIOD->ODR &= ~(1U << 14);
					status = 0;
					break;
			}


		}

}
