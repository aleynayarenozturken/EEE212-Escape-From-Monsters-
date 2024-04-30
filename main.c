#include <MKL25Z4.h>
#include "utils.h"


/*  PTA1 -> Button Interrupt Falling Edge Pull-Up Resistor
		PTA13 -> TPM1_CH1
		
*/




int main (void){
	go();
	init_UltraSensor();
	__enable_irq();
	
	while(1){
		__WFI();
	}
}