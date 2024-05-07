#include <MKL25Z4.h>

#define RS 0x04     /* PTA2 mask */ 
#define RW 0x10     /* PTA4 mask */ 
#define EN 0x20     /* PTA5 mask */

/* PTE22 -> Ultrason trig */

volatile char pattern = 0, mode = 0;
volatile int ultracount = 0, flag = 1;

void Delay(unsigned long long);
void go(void);
void init_UltraSensor(void);
void PORTA_IRQHandler(void);
void SysTick_Handler(void);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_ready(void);


void go(){    /*configure input-output and timers*/
    
	
    SIM->SCGC5 |= 0x0200; /*enable clock to PortA*/
		SIM->SCGC5 |= 0x400; /*enable clock to PortB*/
		PORTB->PCR[8] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORTB->PCR[9] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	  PORTA->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x09); /*make PTA1 GPIO, enable pull-down resistor, interrupt on rising edge*/
	  PORTA->PCR[13] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0x0A);
	  PTA->PDDR &= ~((3UL << 12) | 4); /*make PTA12 and PTA13 input*/
		PTB->PDDR &= ~(3UL << 8);
    NVIC_SetPriority(PORTA_IRQn, 128); /*set interrupt priority*/
    NVIC_ClearPendingIRQ(PORTA_IRQn); /*clear pending interrupts*/    
    NVIC_EnableIRQ(PORTA_IRQn); /*enable IRQ for portA*/
    
		SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; /*start TPM1*/
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); /*set clock source as 41.94MHz*/
    PORTE->PCR[21] |= PORT_PCR_MUX(3); /*make PTE21 as TPM1_CH1*/
    TPM1->SC |= TPM_SC_CMOD(1) | 7; /*set the prescaler to 128*/
    TPM1->CONTROLS[1].CnSC |= 0x0028; /*set CH1 of TPM1 as output*/
    TPM1->CONTROLS[1].CnV = 246; /*set the initial compare value for continuous rotation*/
    TPM1->MOD = 3279; /*set the modulo for TPM1*/
		
		SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; /*start TPM1*/
    PORTC->PCR[8] |= PORT_PCR_MUX(3); /*make PTC8 as TPM0_CH4*/
    TPM0->SC |= TPM_SC_CMOD(1) | 7; /*set the prescaler to 128*/
    TPM0->CONTROLS[4].CnSC |= 0x0028; /*set CH1 of TPM1 as output*/
    TPM0->CONTROLS[4].CnV = 246; /*set the initial compare value for continuous rotation*/
    TPM0->MOD = 3279; /*set the modulo for TPM1*/

    SysTick->CTRL |= 0x0003; /*set clock source, tickint and interrupt enable*/
    SysTick->LOAD = 650000; /*load value for systick*/
    
}

void init_UltraSensor(){
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; /*start TPM2*/
		SIM->SCGC5 |= 0x2000;
    PORTE->PCR[22] |= PORT_PCR_MUX(3); /*make PTE22 as TPM1_CH1*/
    TPM2->SC |= TPM_SC_CMOD(1) | 4; /*set the prescaler to 64  3.05 uS*/
    TPM2->CONTROLS[0].CnSC |= 0x0028; /*set CH0 of TPM2 as output*/
    TPM2->CONTROLS[0].CnV = 13; /*set the initial compare value for continuous rotation*/
    TPM2->MOD = 49971; /*set the modulo for TPM1*/
}

void PORTA_IRQHandler(){
	if(PORTA->ISFR & (1UL << 13)){
		LCD_command(0x01); LCD_data('M');LCD_data('O');LCD_data('D');LCD_data('E');LCD_data(':');LCD_data(' ');
		switch(mode){
			case 0:
				mode = 1;
				LCD_data('2');
				break;
			default:
				mode = 0;
				LCD_data('1');
		}
	}
	if(PORTA->ISFR & (1UL << 12)){
	if (!mode){
	 ultracount = 0;
   while(PTA->PDIR & 0x1000){
			ultracount++;
	 }
	 if(ultracount <= 0x1000){		

			 if(pattern < 2){
				 TPM1->CONTROLS[1].CnV = 281;
				 TPM0->CONTROLS[4].CnV = 211;
			 } else {
				 TPM1->CONTROLS[1].CnV = 211;
				 TPM0->CONTROLS[4].CnV = 281;
			 }
			 Delay(5000000);
			 if(pattern < 3)
					pattern++;
				else
					pattern = 0;
				TPM1->CONTROLS[1].CnV = 246;
				TPM0->CONTROLS[4].CnV = 246;
		 }
	 }
	 }
	 
	 
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

void LCD_init(void)
{
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */ 
    PORTD->PCR[0] = 0x100;      /* make PTD0 pin as GPIO */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PORTD->PCR[2] = 0x100;      /* make PTD2 pin as GPIO */
    PORTD->PCR[3] = 0x100;      /* make PTD3 pin as GPIO */
    PORTD->PCR[4] = 0x100;      /* make PTD4 pin as GPIO */
    PORTD->PCR[5] = 0x100;      /* make PTD5 pin as GPIO */
    PORTD->PCR[6] = 0x100;      /* make PTD6 pin as GPIO */
    PORTD->PCR[7] = 0x100;      /* make PTD7 pin as GPIO */
    PTD->PDDR = 0xFF;           /* make PTD7-0 as output pins */
    
    SIM->SCGC5 |= 0x0200;       /* enable clock to Port A */ 
    PORTA->PCR[2] = 0x100;      /* make PTA2 pin as GPIO */
    PORTA->PCR[4] = 0x100;      /* make PTA4 pin as GPIO */
    PORTA->PCR[5] = 0x100;      /* make PTA5 pin as GPIO */
    PTA->PDDR |= 0x34;          /* make PTA5, 4, 2 as output pins */
    
    LCD_command(0x38);      /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x01);      /* clear screen, move cursor to home */
    LCD_command(0x0F);      /* turn on display, cursor blinking */
	  LCD_data('M');LCD_data('O');LCD_data('D');LCD_data('E');LCD_data(':');LCD_data(' ');LCD_data('1');
}

/* This function waits until LCD controller is ready to
 * accept a new command/data before returns.
 */
void LCD_ready(void)
{
    uint32_t status;
    
    PTD->PDDR = 0x00;          /* PortD input */
    PTA->PCOR = RS;         /* RS = 0 for status */
    PTA->PSOR = RW;         /* R/W = 1, LCD output */
    
    do {    /* stay in the loop until it is not busy */
			  PTA->PCOR = EN;
			  Delay(500);
        PTA->PSOR = EN;     /* raise E */
        Delay(500);
        status = PTD->PDIR; /* read status register */
        PTA->PCOR = EN;
        Delay(500);			/* clear E */
    } while (status & 0x80UL);    /* check busy bit */
    
    PTA->PCOR = RW;         /* R/W = 0, LCD input */
    PTD->PDDR = 0xFF;       /* PortD output */
}

void LCD_command(unsigned char command)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PCOR = RS | RW;    /* RS = 0, R/W = 0 */
    PTD->PDOR = command;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}

void LCD_data(unsigned char data)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PSOR = RS;         /* RS = 1, R/W = 0 */
    PTA->PCOR = RW;
    PTD->PDOR = data;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}
