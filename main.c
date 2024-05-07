#include <MKL25Z4.h>
#include "utils.h"


/*  PTA1 -> Button Interrupt Falling Edge Pull-Up Resistor
		PTA13 -> TPM1_CH1
		PTC8 -> TPM0_CH4
*/




int main (void){
	go();
	init_UltraSensor();
	LCD_init();
	__enable_irq();
	
	while(1){
		if(mode){
			switch(PTB->PDIR & 0x300){
				case 0x100:
					TPM1->CONTROLS[1].CnV = 281;
					TPM0->CONTROLS[4].CnV = 211;
					break;
				case 0x200:
					TPM1->CONTROLS[1].CnV = 211;
					TPM0->CONTROLS[4].CnV = 281;
					break;
				default:
					TPM1->CONTROLS[1].CnV = 246;
					TPM0->CONTROLS[4].CnV = 246;
			}
		}
	}
}