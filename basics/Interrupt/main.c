#include "msp.h"

void gpio_cfg(void){
	// P1.01 and P1.04 Button as Input
	P1->SEL0 &= ~(BIT1 | BIT4);
  P1->SEL1 &= ~(BIT1 | BIT4);
  P1->DIR &= ~(BIT1 | BIT4);
  P1->REN |= (BIT1 | BIT4);
  P1->OUT |= (BIT1 | BIT4);

	// Led's as output
  P2->SEL0 &= ~(BIT0 | BIT1);
  P2->SEL1 &= ~(BIT0 | BIT1);
  P2->DIR |= (BIT0 | BIT1);

}

void int_cfg(void){
	P1->IFG = 0; 			// Clear interrupt flags
	P1->IES |= (BIT1 | BIT4); 	// P1.1 and P1.4 high-low transition
	P1->IE |= (BIT1 | BIT4); 	// Enable the interrupt on these ports
	NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31); 		// set IRQ35->P1 (bit 35)
}

void main(void)
{
	int i = 0;
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	
	gpio_cfg();
	int_cfg();

	while(1){
		__sleep();
		i++;
	}
}

void PORT1_IRQHandler(void){
	
	if(P1->IFG & BIT1){ 		// Check if there was a trigger in PIN1
		P2->OUT ^= (BIT0);		// Toggle led
		P1->IFG &= (~BIT1); 	// Clear interrupt flag
	}

	if(P1->IFG & BIT4){			// Check if there was a trigger in PIN1
		P2->OUT ^= (BIT1);		// Toggle led
		P1->IFG &= (~BIT4); 	// Clear interrupt flag
	}
}
