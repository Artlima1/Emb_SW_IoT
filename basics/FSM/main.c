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

void fn_init(void);
void fn_green(void);
void fn_red(void);
void fn_blue(void);

State_t cur_state = ST_INIT;

StateMachine_t StateMachine[] = {
	{ST_INIT, fn_init}, 
	{ST_RED, fn_red},
	{ST_GREEN, fn_green},
	{ST_BLUE, fn_blue}
};

void configGPIO(){
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

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	configGPIO();
	while(1){
		if(cur_state < NUM_OF_STATES){
			(*StateMachine[cur_state].func)();
		}
		else{
			// error 
		}
	}

}

void check_s1(void){
	while(P1->IN & S1);       // Wait for press
	while(!(P1->IN & S1));    // Wait for release
}

void check_s2(void){
	while(P1->IN & S2);       // Wait for press
	while(!(P1->IN & S2));    // Wait for release
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
	check_s1();
	cur_state = ST_GREEN;
}

void fn_green(void){
	green_on();
	check_s2();
	cur_state = ST_RED;
}

void fn_red(void){
	red_on();
	check_s1();
	cur_state = ST_BLUE;
}

void fn_blue(void){
	blue_on();
	check_s2();
	cur_state = ST_GREEN;
}
