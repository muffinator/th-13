/* This code is for the whit DHT22 sensor */
/* The blue sensor has slightly different byte format */
/* but the humidity numbers are too bad from it to use for anything */
/* could make an ok temp sensor but you are better off rolling your own with a thermistor */
/* opamp and LUT */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "contiki.h"
#include "th-12.h"
#include "current.h"

#include "mc1322x.h"

/* debug */
/* commented out since it can conflict with the debug setting in th-12.c */
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h" 

#define setdo(x) GPIO->PAD_DIR_SET.x=1
#define setdi(x) GPIO->PAD_DIR_RESET.x=1

#define MAX_SAMPLES 256
uint16_t current_data[MAX_SAMPLES]; /* dht11 returns 40 pulses, allocate a little extra just in case */
volatile uint8_t current_idx;                /* current index into the results buffer */
current_result_t c;


void tmr1_isr(void) {
	if( TMR1->CSCTRLbits.TCF1 == 1)
	{
		c.data[current_idx] = ADC->RESULT_1 & 0xfff;
		current_idx++;
		if(current_idx>255){
			current_idx = 0;
			TMR1->CTRL &= ~(1<13); //turn off timer
		}
	}
	TMR1->SCTRLbits.TCF = 0;
	TMR1->CSCTRLbits.TCF1 = 0;
}



void tmr1_init(void) {
    //  Start samplerate timer.
	#define TMR1_STOP           0       // Stop
	#define TMR1_RUN            1       // Count rising edges of primary source
	#define TMR1_PRIME_SRC      0xb     // Peripheral Clock divided by 8 prescaler
	#define TMR1_SEC_SRC        0       // Don't need this
	#define TMR1_ONCE           0       // Count until compare and then count again
	#define TMR1_LEN            1       // Count until compare then reload with value in LOAD
	#define TMR1_DIR            0       // Count up
	#define TMR1_CO_INIT        0       // Other counters can not reinitialize
	#define TMR1_OUT_MODE       3       // Toggle on successful compare
    TMR1->CTRL = (TMR1_STOP<<13) | (TMR1_PRIME_SRC<<9) | (TMR1_SEC_SRC<<7) | (TMR1_ONCE<<6) |(TMR1_LEN<<5) | (TMR1_DIR<<4) | (TMR1_CO_INIT<<3) | (TMR1_OUT_MODE);
    TMR1->SCTRL     = 1;        // External pin configured as output
    TMR1->CSCTRL    = 0x0041;   // Interrupt on compare.  Load w/successful compare with the value in COMP1 
    TMR1->LOAD		= 0;        // reload to zero
    TMR1->COMP1		= 195;//POWSAMPLECNTR;
    TMR1->COMP2		= 0;
    TMR1->CMPLD1	= 195;//POWSAMPLECNTR;
    TMR1->CMPLD2	= 0;
    TMR1->CNTR		= 0;        // reset count register
    TMR1->CTRL		= (TMR1_STOP<<13) | (TMR1_PRIME_SRC<<9) | (TMR1_SEC_SRC<<7) | (TMR1_ONCE<<6) | (TMR1_LEN<<5) | (TMR1_DIR<<4) | (TMR1_CO_INIT<<3) | (TMR1_OUT_MODE);
}

void adc_setup(void) {
	// setup adc0 pin
    GPIO->FUNC_SEL.ADC0 = 1;
    GPIO->PAD_DIR.ADC0 = 0;
    GPIO->PAD_KEEP.ADC0 = 0;
    GPIO->PAD_PU_EN.ADC0 = 0;

	// setup adc registers
    ADC->CONTROL = 0;               // no sequences or interrupts
    ADC->CONTROLbits.ON = 1;        // ADC on
    ADC->CLOCK_DIVIDER = 80;        // 300KHz
    ADC->PRESCALE = 23;             // 1MHz
    ADC->ON_TIME = 10;              // 10us 
    ADC->CONVERT_TIME = 40;         // 40us             
    ADC->MODE = 1;                  // manual mode
    ADC->OVERRIDEbits.MUX1 = 0;     // ADC1 sampling input 0
    ADC->OVERRIDEbits.MUX2 = 0;     // ADC2 sampling input 1
    ADC->OVERRIDEbits.AD1_ON = 1;   // ADC1 on
    ADC->OVERRIDEbits.AD2_ON = 1;   // ADC2 on
}

/* signals the dht to send data back */
/* waits for the result */
/* posts a dht_done event with a dht result struct in data */

struct etimer et_dht;

void (*current_result)(current_result_t c);

void register_current_result( void (*current_result_cb) ) {
	current_result = current_result_cb;
}

PROCESS(read_current, "read current");
PROCESS_THREAD(read_current, ev, data)
{
	c.ok=1;
	uint8_t bit_offset;
	PROCESS_BEGIN();

	TMR1->CTRL |= (TMR1_RUN<<13);
	
#ifdef DEBUG_FULL
#endif

#ifdef DEBUG_FULL
#endif
	PRINTF("current process");			
		current_result(c);

		PROCESS_EXIT();
	PROCESS_END();
}	
				

void current_init(void)
{
	/* power on mid-rail reference */
	setdo(KBI2);
	GPIO->FUNC_SEL.KBI2=3;
	gpio_set(KBI2);

	/* power on diff amp */
	setdo(KBI1);
	GPIO->FUNC_SEL.KBI1=3;
	gpio_set(KBI1);

	/* trip opto-relay */
	setdo(KBI0);
	GPIO->FUNC_SEL.KBI0=3;
	gpio_set(KBI0);

	tmr1_init();	
	adc_setup();
	enable_irq(TMR);	
}

void current_uninit(void)
{
	CRM->WU_CNTLbits.EXT_OUT_POL = 0; /* drive KBI0-3 low during sleep */
	gpio_reset(KBI2);
	gpio_reset(KBI1);
	gpio_reset(KBI0);
}
