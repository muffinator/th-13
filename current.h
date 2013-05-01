#ifndef __SEN_H__
#define __SEN_H__

#include "mc1322x.h"

#define BURD_EN(void) gpio_set(KBI0)
#define BURD_DIS(void) gpio_reset(KBI0)

#define REF_EN(void) gpio_set(KBI2)
#define REF_DIS(void) gpio_set(KBI2)

/* high impedance with pullup */
#define DHT_PU(void) \
	GPIO->PAD_DIR_RESET.TMR1=1;\
	GPIO->FUNC_SEL.TMR1=1;\


typedef struct current_result {
	volatile uint16_t data[256];
	uint16_t rh; /* relative humidity in % * 10 */
	int16_t t;   /* temp in C * 10 */
	uint8_t ok;  /* equals 1 if checksum was ok */
} current_result_t;

PROCESS_NAME(read_current);

void current_init(void);

/* register a function to be called when the dht has a result */
/* the callback takes a dht_result_t */
/* void (*dht_result)(dht_result_t d); */
void register_current_result( void (*current_result_cb) );

#endif /*__SEN_H__*/
