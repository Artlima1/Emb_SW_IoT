#include "msp.h"

void configGPIO_v1() {
  // Selecting P1.01 as GPIO
  P1->SEL0 &= ~BIT1;
  P1->SEL1 &= ~BIT1;
  // Setting P1.01 in input mode
  P1->DIR &= ~BIT1;
  P1->REN |= BIT1;
  // Activating pullup resistor for P1.01
  P1->OUT |= BIT1;

  // Selecting P2.01 as GPIO
  P2->SEL0 &= ~BIT1;
  P2->SEL1 &= ~BIT1;
  // Setting P2.01 in output mode
  P2->DIR |= BIT1;
}

void configGPIO_v2(){
  // Input
  P1->SEL0 &= ~(BIT1 | BIT4);
  P1->SEL1 &= ~(BIT1 | BIT4);

  P1->DIR &= ~(BIT1 | BIT4);
  P1->REN |= (BIT1 | BIT4);

  P1->OUT |= (BIT1 | BIT4);

  // Output
  P2->SEL0 &= ~(BIT1 | BIT2);
  P2->SEL1 &= ~(BIT1 | BIT2);

  P2->DIR |= (BIT1 | BIT2);
}

void v1() {
  configGPIO_v1();

  while (1) {
    while (P1->IN & BIT1);      // Wait for press
    P2->OUT ^= BIT1;            // Toggle P2.1
    while (!(P1->IN & BIT1));   // Wait for release
    P2->OUT ^= BIT1;            // Toggle P2.01
  }
}

void v2() {
  configGPIO_v2();

  while (1) {
    while ((P1->IN & BIT1) && (P1->IN & BIT4));  // Wait for press

    if (!(P1->IN & BIT1)) {
      P2->OUT ^= BIT1;                          // Toggle P2.1
      while (!(P1->IN & BIT1));                 // Wait for release
      P2->OUT ^= BIT1;                          // Toggle P2.01
    } else {
      P2->OUT ^= BIT4;                          // Toggle P2.4
      while (!(P1->IN & BIT2));                 // Wait for release
      P2->OUT ^= BIT4;                          // Toggle P2.04
    }
  }
}

void main(void) {
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // stop watchdog timer

  v1();
  //    v2();
}
