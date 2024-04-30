#include <MKL25Z4.h>

/* PTE22 -> Ultrason trig */

volatile char pattern = 0;
int ultracount = 0, buffer;

void Delay(unsigned long long);
void go(void);
void init_UltraSensor(void);
void PORTA_IRQHandler(void);
void SysTick_Handler(void);

void go(){    /*configure input-output and timers*/
    
    SIM->SCGC5 |= 0x0200; /*enable clock to PortA*/
    PTA->PDDR &= ~1UL; /*make PTA1 input*/
    PORTA->PCR[1] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x09); /*make PTA1 GPIO, enable pull-down resistor, interrupt on rising edge*/
    NVIC_SetPriority(PORTA_IRQn, 128); /*set interrupt priority*/
    NVIC_ClearPendingIRQ(PORTA_IRQn); /*clear pending interrupts*/    
    NVIC_EnableIRQ(PORTA_IRQn); /*enable IRQ for portA*/
    
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; /*start TPM1*/
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); /*set clock source as 41.94MHz*/
    PORTA->PCR[13] |= PORT_PCR_MUX(3); /*make PTA13 as TPM1_CH1*/
    TPM1->SC |= TPM_SC_CMOD(1) | 7; /*set the prescaler to 128*/
    TPM1->CONTROLS[1].CnSC |= 0x0028; /*set CH1 of TPM1 as output*/
    TPM1->CONTROLS[1].CnV = 246; /*set the initial compare value for continuous rotation*/
    TPM1->MOD = 3279; /*set the modulo for TPM1*/

    SysTick->CTRL |= 0x0003; /*set clock source, tickint and interrupt enable*/
    SysTick->LOAD = 650000; /*load value for systick*/
    
}

void init_UltraSensor(){
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; /*start TPM1*/
		SIM->SCGC5 |= 0x2000;
    PORTE->PCR[22] |= PORT_PCR_MUX(3); /*make PTA13 as TPM1_CH1*/
    TPM2->SC |= TPM_SC_CMOD(1) | 4; /*set the prescaler to 64  3.05 uS*/
    TPM2->CONTROLS[0].CnSC |= 0x0028; /*set CH1 of TPM1 as output*/
    TPM2->CONTROLS[0].CnV = 13; /*set the initial compare value for continuous rotation*/
    TPM2->MOD = 49971; /*set the modulo for TPM1*/
}

void PORTA_IRQHandler(){
	 ultracount = 0;
   while(PTA->PDIR & 0x1){
			ultracount++;
	 }
	 if(ultracount <= 0x50){
		 TPM1->CONTROLS[1].CnV = 281;
		 Delay(50);
		 TPM1->CONTROLS[1].CnV = 246;
	 }
	 buffer = ultracount;
    NVIC_ClearPendingIRQ(PORTA_IRQn); /*clear pending interrupts for port A*/
    PORTA->ISFR = ~0UL; /*set 1 of all bits of ISFR*/
}

void SysTick_Handler(){
    /* No need to change PWM settings in SysTick, just let it run continuously */
}

void Delay(unsigned long long T){
	while(T--){
	}
}