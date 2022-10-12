#include "msp.h"

#define RED_LED BIT0
#define GREEN_LED BIT1
#define BLUE_LED BIT2
#define S1 BIT1
#define S2 BIT4

typedef enum {
	ST_INIT, 
	ST_RED,
	ST_GREEN,
	ST_BLUE,
	NUM_OF_STATES,
} State_t;

typedef struct {
	State_t state;
	void (*func)(void);
} StateMachine_t;

typedef enum {
	NONE,
	SW1_PRESSED, 
	SW2_PRESSED
} Event_t;

void fn_init(void);
void fn_green(void);
void fn_red(void);
void fn_blue(void);

State_t cur_state = ST_INIT;
Event_t event = NONE;

StateMachine_t StateMachine[] = {
	{ST_INIT, fn_init}, 
	{ST_RED, fn_red},
	{ST_GREEN, fn_green},
	{ST_BLUE, fn_blue}
};

void configGPIO(void){
  // Input
	P1->SEL0 &= ~(S1 | S2);
	P1->SEL1 &= ~(S1 | S2);

	P1->DIR &= ~(S1 | S2);
	P1->REN |= (S1 | S2);

	P1->OUT |= (S1 | S2);

	// Output
	P2->SEL0 &= ~(GREEN_LED | RED_LED | BLUE_LED);
	P2->SEL1 &= ~(GREEN_LED | RED_LED | BLUE_LED);

	P2->DIR |= (GREEN_LED | RED_LED | BLUE_LED);
}

void configInterrupt(void){
	P1->IFG = 0; 			// Clear interrupt flags
	P1->IES |= (BIT1 | BIT4); 	// P1.1 and P1.4 high-low transition
	P1->IE |= (BIT1 | BIT4); 	// Enable the interrupt on these ports
	NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31); 		// set IRQ35->P1 (bit 35)
}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	configGPIO();
	configInterrupt();

	while(1){
		__sleep();
		if(cur_state < NUM_OF_STATES){
			(*StateMachine[cur_state].func)();
		}
		else{
			// error 
		}
	}

}

void green_on(){
    P2->OUT &= ~(GREEN_LED | RED_LED | BLUE_LED);
    P2->OUT |= GREEN_LED;
}

void red_on(){
    P2->OUT &= ~(GREEN_LED | RED_LED | BLUE_LED);
    P2->OUT |= RED_LED;
}

void blue_on(){
    P2->OUT &= ~(GREEN_LED | RED_LED | BLUE_LED);
    P2->OUT |= BLUE_LED;
}

void fn_init(void){
	if(event == SW1_PRESSED){
		green_on();
		cur_state = ST_GREEN;
	}
}

void fn_green(void){
	if(event == SW2_PRESSED){
		red_on();
		cur_state = ST_RED;
	}
}

void fn_red(void){
	if(event == SW1_PRESSED){
		blue_on();
		cur_state = ST_BLUE;
	}
}

void fn_blue(void){
	if(event == SW2_PRESSED){
		green_on();
		cur_state = ST_GREEN;
	}
}

void PORT1_IRQHandler(void){
	
	if(P1->IFG & BIT1){ 		// Check if there was a trigger in PIN1
		event = SW1_PRESSED;	// Set event flag
		P1->IFG &= (~BIT1); 	// Clear interrupt flag
	}

	if(P1->IFG & BIT4){			// Check if there was a trigger in PIN1
		event = SW2_PRESSED;	// Set event flag
		P1->IFG &= (~BIT4); 	// Clear interrupt flag
	}
}
