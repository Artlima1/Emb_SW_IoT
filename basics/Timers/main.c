#include "msp.h"

void timer_cfg(void){
	TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; 				// enable interrupt trigger on timer HW
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |			// set the clock source
									TIMER_A_CTL_MC__CONTINUOUS |	// set operation mode of the timer
									TIMER_A_CTL_ID_3;							// set the pre-scaler (reduce frequency of the signal)
	TIMER_A0->CCR[0] = 50000; 										// load the comparator to generate interrupt
	
	NVIC->ISER[0] = 1 <<((TA0_0_IRQn) & 31); 			// enable interrupt service
}

void gpio_cfg(){
	P2->SEL0 &= ~BIT1;
  P2->SEL1 &= ~BIT1;
  P2->DIR |= BIT1;
}

void init(){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	gpio_cfg();
	timer_cfg();
}

void main(void)
{

	init();

	while (1)
	{
		__sleep();
	}
	
}


void TA0_0_IRQHandler(void){
	TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // clear interrupt

	P2->OUT ^= BIT1;
}
