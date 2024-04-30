#include <MKL25Z4.h>

volatile char pattern = 0;
volatile char angle = 0;

void Delay(int);
void go(void);
void PORTA_IRQHandler(void);
void SysTick_Handler(void);


void go(){ 	/*configure input-output and timers*/
	
	SIM->SCGC5 |= 0x0200; /*enable clock to PortA*/
	PTA->PDDR &= ~1UL; /*make PTA1 input*/
	PORTA->PCR[1] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0x0A); /*make PTA1 GPIO, enable pull-up resistor, interrupt on falling edge*/
	NVIC_SetPriority(PORTA_IRQn, 128); /*set interrupt priority*/
	NVIC_ClearPendingIRQ(PORTA_IRQn); /*clear pending interrupts*/	
	NVIC_EnableIRQ(PORTA_IRQn); /*enable IRQ for portA*/
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; /*start TPM1*/
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); /*set clock source as 41.94MHz*/
	PORTA->PCR[13] |= PORT_PCR_MUX(3); /*make PTA13 as TPM1_CH1*/
	TPM1->SC |= TPM_SC_CMOD(1) | 7; /*set the prescaler to 128*/
	TPM1->CONTROLS[1].CnSC |= 0x0028; /*set CH1 of TPM1 as output*/
	TPM1->CONTROLS[1].CnV = 83; /*set the initial compare value*/
	TPM1->MOD = 3279; /*set the modulo for TPM1*/

	SysTick->CTRL |= 0x0003; /*set clock source, tickint and interrupt enable*/
	SysTick->LOAD = 650000; /*load value for systick*/
	
}

void PORTA_IRQHandler(){
	pattern++; /*increment direction flag*/
	NVIC_ClearPendingIRQ(PORTA_IRQn); /*clear pending interrupts for port A*/
	PORTA->ISFR = ~0UL; /*set 1 of all bits of ISFR*/
}

void SysTick_Handler(){
	if(angle < 7)
		++angle;
	else 
		angle = 0;
	if(pattern & 1){
		switch(angle){
			case 0:
				TPM1->CONTROLS[1].CnV = 83; 
				break;
			case 1:
				TPM1->CONTROLS[1].CnV = 136; 
				break;
			case 2:
				TPM1->CONTROLS[1].CnV = 192; 
				break;
			case 3:
				TPM1->CONTROLS[1].CnV = 245; 
				break;
			case 4:
				TPM1->CONTROLS[1].CnV = 192; 
				break;
			case 5:
				TPM1->CONTROLS[1].CnV = 136; 
				break;
			case 6:
				TPM1->CONTROLS[1].CnV = 83; 
				break;
		}
	} 
	else {
		switch(angle){
			case 0:
				TPM1->CONTROLS[1].CnV = 410; 
				break;
			case 1:
				TPM1->CONTROLS[1].CnV = 356; 
				break;
			case 2:
				TPM1->CONTROLS[1].CnV = 301; 
				break;
			case 3:
				TPM1->CONTROLS[1].CnV = 245; 
				break;
			case 4:
				TPM1->CONTROLS[1].CnV = 301; 
				break;
			case 5:
				TPM1->CONTROLS[1].CnV = 356; 
				break;
			case 6:
				TPM1->CONTROLS[1].CnV = 410; 
				break;
		}
	}

}