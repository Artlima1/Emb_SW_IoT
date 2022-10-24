/* DriverLib Includes */
#include "msp.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* ------------------- SOME MACROS ---------------------------- */
#define RED_LED GPIO_PIN0
#define GREEN_LED GPIO_PIN1
#define BLUE_LED GPIO_PIN2
#define LEDS_PORT GPIO_PORT_P1

/* ------------------- Configuration of FSM ---------------------------- */

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

/* ------------------- Temperature Variables ---------------------------- */

uint32_t cal30;
uint32_t cal85;
float calDifference;

float temperature;

/* ------------------- Initialization of elements ---------------------------- */

void config_sensor(void){
	REF_A_enableTempSensor();
    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    REF_A_enableReferenceVoltage();

    cal30 = SysCtl_getTempCalibrationConstant(SYSCTL_2_5V_REF, SYSCTL_30_DEGREES_C);
    cal85 = SysCtl_getTempCalibrationConstant(SYSCTL_2_5V_REF, SYSCTL_85_DEGREES_C);
    calDifference = cal85 - cal30;
}

void config_adc(void){
	ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_TEMPSENSEMAP);

    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A22, false);

    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_192,ADC_PULSE_WIDTH_192);

    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_enableInterrupt(ADC_INT0);

    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}

void config_gpio(void){
	GPIO_setAsOutputPin(LEDS_PORT, RED_LED);
	GPIO_setAsOutputPin(LEDS_PORT, GREEN_LED);
	GPIO_setAsOutputPin(LEDS_PORT, BLUE_LED);
}

/* ------------------- Main Program ---------------------------- */

void init(void){
	WDT_A_holdTimer();
	config_sensor();
    config_adc();
    config_gpio();
}

int main(void)
{
	init();

    while (1)
    {
        PCM_gotoLPM0();
    }
}

/* ------------------- Helper Functions ---------------------------- */

void read_temperature(void){
	int16_t conRes;
	conRes = ((ADC14_getResult(ADC_MEM0) - cal30) * 55);
	temperature = (conRes / calDifference) + 30.0f;
}

/* ------------------- Interrupt handlers and state definitions ---------------------------- */

void fn_init(void){
	if(temperature > 
}

void fn_green(void){

}

void fn_red(void){

}

void fn_blue(void){

}


void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    if(status & ADC_INT0)
    {
        read_temperature();

        /* TODO */

    }

}