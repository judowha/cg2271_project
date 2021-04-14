/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <math.h>
#define MOTOR_CONTORL_RIGHT_forward 2 //PortA pin 2 TPM2_CH1
#define MOTOR_CONTORL_RIGHT_backward 4// PORTA pin 4 GPIO
#define MOTOR_CONTORL_LEFT_forward 1 //PortA pin 1 TPM2_CH0
#define MOTOR_CONTORL_LEFT_backward 5// PORTA pin 5 GPIO
#define MASK(x) (1 << (x))
#define UART_RX_PORTE23 23 //uart receive
#define BAUD_RATE 9600
//#define BLUE_LED 1
//LED PART
#define rear_LED 8
#define front_LED1 20
#define front_LED2 21
#define front_LED3 22
#define front_LED4 29
#define front_LED5 0
#define front_LED6 1
#define front_LED7 2
#define front_LED8 3
#define front_blinking_LED 1

//sound
#define PTD0_Pin 0 //Port A pin 12 TPM1_CH0 for buzzer pin
#define DEGREE_CNT 25	//USED?
#define TO_MOD(x) 375000/(x)	//USED?
#define DEGREE_B0 31
#define DEGREE_C1 33
#define DEGREE_CR1 35
#define DEGREE_D1 37
#define DEGREE_DR1 39
#define DEGREE_E1 41
#define DEGREE_F1 44
#define DEGREE_FR1 46
#define DEGREE_G1 49
#define DEGREE_GR1 52
#define DEGREE_A1 55
#define DEGREE_AR1 58
#define DEGREE_B1 62
#define DEGREE_C2 65
#define DEGREE_CR2 69
#define DEGREE_D2 73
#define DEGREE_DR2 78
#define DEGREE_E2 82
#define DEGREE_F2 87
#define DEGREE_FR2 93
#define DEGREE_G2 98
#define DEGREE_GR2 104
#define DEGREE_A2 110
#define DEGREE_AR2 117
#define DEGREE_B2 123
#define DEGREE_C3 131
#define DEGREE_CR3 139
#define DEGREE_D3 147
#define DEGREE_DR3 156
#define DEGREE_E3 165
#define DEGREE_F3 175
#define DEGREE_FR3 185
#define DEGREE_G3 196
#define DEGREE_GR3 208
#define DEGREE_A3 220
#define DEGREE_AR3 233
#define DEGREE_B3 247
#define DEGREE_C4 262
#define DEGREE_CR4 277
#define DEGREE_D4 294
#define DEGREE_DR4 311
#define DEGREE_E4 330
#define DEGREE_F4 349
#define DEGREE_FR4 370
#define DEGREE_G4 392
#define DEGREE_GR4 415
#define DEGREE_A4 440
#define DEGREE_AR4 466
#define DEGREE_B4 494
#define DEGREE_C5 523
#define DEGREE_CR5 554
#define DEGREE_D5 587
#define DEGREE_DR5 622
#define DEGREE_E5 659
#define DEGREE_F5 698
#define DEGREE_FR5 740
#define DEGREE_G5 784
#define DEGREE_GR5 831
#define DEGREE_A5 880
#define DEGREE_AR5 932
#define DEGREE_B5 988
#define DEGREE_C6 1047
#define DEGREE_CR6 1109
#define DEGREE_D6 1175
#define DEGREE_DR6 1245
#define DEGREE_E6 1319
#define DEGREE_F6 1397
#define DEGREE_FR6 1480
#define DEGREE_G6 1568
#define DEGREE_GR6 1661
#define DEGREE_A6 1760
#define DEGREE_AR6 1865
#define DEGREE_B6 1976
#define DEGREE_C7 2093
#define DEGREE_CR7 2217
#define DEGREE_D7 2349
#define DEGREE_DR7 2489
#define DEGREE_E7 2637
#define DEGREE_F7 2794
#define DEGREE_FR7 2960
#define DEGREE_G7 3136
#define DEGREE_GR7 3322
#define DEGREE_A7 3520
#define DEGREE_AR7 3729
#define DEGREE_B7 3951
#define DEGREE_C8 4186
#define DEGREE_CR8 4435
#define DEGREE_D8 4699
#define DEGREE_DR8 4978
#define REST 375000
int tempo = 144;
int melody[] = {
	REST,4, REST,4, REST,8, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//L1 1
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,4, DEGREE_E6,4,	//2
	DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,4, DEGREE_D6,4,	//3
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,4, DEGREE_C6,4,	//4
	DEGREE_G5,8, DEGREE_G5,8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//5
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_G6,4,	//6
	DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_D6,8, DEGREE_C6,8,	//7
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,1,	//L2 8
	REST,4, DEGREE_D6,8, DEGREE_E6,8,	//9
	DEGREE_D6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,1,	//10
	REST,4, DEGREE_D6,8, DEGREE_E6,8,	//11
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_C6,2,	//12
	REST,4, REST,4, REST,4, REST,4, DEGREE_G5,8,	//13
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8,	//14
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_G5,2, DEGREE_G5,8,	//15
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8,	//16
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_G5,2,  DEGREE_G5,8,	//L3 17
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8,	//18
	DEGREE_D6,8, DEGREE_G5,8, DEGREE_G5,8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_G5,8, DEGREE_E5,8,	//19
	DEGREE_G5,4, DEGREE_A5,8, DEGREE_A5,8, DEGREE_A5,1,	//20
	DEGREE_G5,2,	//21
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8,	//22
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_B5,8, DEGREE_G5,2, DEGREE_G5,8,	//L4 23
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8,	//24
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_B5,8, DEGREE_G5,4, DEGREE_D6,8, DEGREE_E6,8,	//25
	DEGREE_C6,4, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,6, DEGREE_C6,6, DEGREE_C6,6,	//26
	DEGREE_D6,-8, DEGREE_G5,-8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_G5,8, DEGREE_E5,8,	//27
	DEGREE_G5,4, DEGREE_A5,8, DEGREE_A5,2, REST,8,	//28
	REST,1,	//29
	DEGREE_A5,2, DEGREE_A5,8, DEGREE_A5,8, DEGREE_G5,8,	//L5 30
	DEGREE_A5,8, DEGREE_B5,8, DEGREE_B5,8, DEGREE_B5,4, DEGREE_B5,8, DEGREE_B5,8, DEGREE_B5,8,	//31
	DEGREE_B5,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,4, DEGREE_G5,8, DEGREE_G5,8, DEGREE_E5,8,	//32
	DEGREE_G5,8, DEGREE_B5,8, DEGREE_B5,8, DEGREE_B5,4, DEGREE_G5,4,	//33
	DEGREE_B5,2, DEGREE_B5,8, DEGREE_B5,8, DEGREE_G5,8,	//34
	DEGREE_B5,8, DEGREE_A5,8, DEGREE_A5,8, DEGREE_A5,4, DEGREE_A5,8, DEGREE_A5,8, DEGREE_B5,8,	//35
  DEGREE_A5,8, DEGREE_A5,8, DEGREE_C6,8, DEGREE_C6,2,	//36
	DEGREE_E5,-4, DEGREE_D5,24, DEGREE_E5,24, DEGREE_D5,24, DEGREE_A4,8, DEGREE_A4,8, DEGREE_D5,8, DEGREE_E5,8,	//37
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_A5,4, DEGREE_D6,8, DEGREE_E6,8,	//L6 38
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_A5,4, DEGREE_D6,8, DEGREE_E6,8,	//39
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_G5,8, DEGREE_E5,8,	//40
	DEGREE_G5,8, DEGREE_G5,8, DEGREE_A5,8, DEGREE_A5,-4, DEGREE_E6,4,	//41
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_A5,4, DEGREE_C6,8, DEGREE_D6,8, DEGREE_E6,8,	//42
	DEGREE_D6,8, DEGREE_C6,4, DEGREE_A5,8, DEGREE_A5,8, DEGREE_A5,8, DEGREE_D6,4,	//43
	DEGREE_C6,2, REST,8, DEGREE_F5,8, DEGREE_G5,8, DEGREE_C6,8,	//44
	DEGREE_D5,2, REST,8, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//45
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,4, DEGREE_E6,4,	//L7 46
	DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,4, DEGREE_D6,4,	//47
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,4, DEGREE_C6,4,	//48
	DEGREE_G5,8, DEGREE_G5,8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//49
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_G6,4,	//50
	DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_D6,8, DEGREE_C6,8,	//51
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_E6,1,	//52
	REST,4, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//53
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,4, DEGREE_E6,4,	//L8 54
	DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,8, DEGREE_D6,4, DEGREE_D6,4,	//55
	DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,8, DEGREE_C6,4, DEGREE_C6,4,	//56
	DEGREE_G5,8, DEGREE_G5,8, DEGREE_G5,4, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8,	//57
	DEGREE_D6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_G6,4,	//58
	DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_E6,8, DEGREE_D6,8, DEGREE_C6,8,	//59
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_E6,1,	//60
	REST,8, REST,2,	//61
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_C6,8, DEGREE_G5,8, DEGREE_D5,8, DEGREE_C5,8, DEGREE_D5,8, DEGREE_E5,8,	//L9 62
	DEGREE_D5,4, DEGREE_C5,8, DEGREE_A4,2,	//63
	REST,4, DEGREE_A4,8, DEGREE_B4,8, DEGREE_C5,8, DEGREE_G5,8, DEGREE_C6,8, DEGREE_D6,8, DEGREE_E6,8,	//64
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_E6,2,	//65
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_C6,8, DEGREE_G5,8, DEGREE_D5,8, DEGREE_C5,8, DEGREE_D5,8, DEGREE_E5,8,	//66
	DEGREE_D5,4, DEGREE_C5,8, DEGREE_A4,2,	//67
	REST,2, REST,4, DEGREE_D6,8, DEGREE_E6,8,	//68
	DEGREE_D6,4, DEGREE_C6,8, DEGREE_C6,2,	//69
	REST,1,	//70

};
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 osSemaphoreId_t mySem;
 osMutexId_t myMutex;
 uint8_t volatile rx_data ;
 osThreadId_t forward_id, backward_id, left_id, right_id, stop_id, rear_LED_moving_id , 
							rear_LED_stopped_id, front_LED_moving_id, front_LED_stopped_id, 
							front_LED_blinking_id, running_buzzer_id;
							
							

 
 void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~(UART_C2_RE_MASK & UART_C2_RIE_MASK);
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1=0;
	UART2->C2=0;
	UART2->C3=0;
	UART2->C4 &= ~(UART_C4_RDMAS_MASK);
	
	UART2->C2 |= (UART_C2_RE_MASK );
	NVIC_SetPriority(UART2_IRQn,128);
	NVIC_ClearPendingIRQ (UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= (UART_C2_RIE_MASK );
	
	
}

/*
uint8_t UART2_Receive_Poll(void){
	while(!(UART2->S1 & UART_S1_RDRF_MASK))ADC0;
	return (UART2->D);
}
 
*/
void initPWM(void){
	
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK) ;
	PORTA->PCR[MOTOR_CONTORL_RIGHT_forward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_RIGHT_forward] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[MOTOR_CONTORL_LEFT_forward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_LEFT_forward] |= PORT_PCR_MUX(3);
	
	//PORTA->PCR[buzzer] &= ~PORT_PCR_MUX_MASK;
	//PORTA->PCR[buzzer] |= PORT_PCR_MUX(3);
	
	
	SIM->SCGC5 = (SIM_SCGC5_PORTD_MASK);
// Configure Mode 3 for PWM pin operation
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
//PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;
//PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);
//Enable clock gating for Timer1
	SIM->SCGC6 = (SIM_SCGC6_TPM0_MASK);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

TPM0->MOD = 7500;
//Edge-Aligned PWM
//CMOD - 1 and PS - 111 (128)
TPM0_SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
TPM0_SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //CMOD = 1 => LPTPM counter increments on every LPTPM
//counter clock
TPM0_SC &= ~(TPM_SC_CPWMS_MASK); //count up by default (0)
//enable PWM on TPM0 channel 0 - PTD0
TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) |
(TPM_CnSC_MSA_MASK));
TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
//enable PWM on TPM0 channel 1 - PTD1
TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) |
(TPM_CnSC_MSA_MASK));
TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1->MOD = 7500;
	TPM1->SC &= ~( TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
	TPM1->SC |= (TPM_SC_CMOD(1)| TPM_SC_PS(7) );
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
	TPM1_C0SC	|= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
	TPM1_C1SC	|= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM2->MOD = 7500;
	TPM2->SC &= ~( TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
	TPM2->SC |= (TPM_SC_CMOD(1)| TPM_SC_PS(7) );
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
	TPM2_C0SC	|= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM2_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
	TPM2_C1SC	|= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM2_C1V = 0;
	TPM2_C0V = 0;
	
	TPM1_C1V = 0;
	TPM1_C0V = 0;
}



 void initGPI0(){
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTA_MASK));
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	 
	//PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	//PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	//PTD->PDDR |= MASK(BLUE_LED);
	//initialize motors
	PORTA->PCR[MOTOR_CONTORL_RIGHT_backward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_RIGHT_backward] |= PORT_PCR_MUX(1);
	PORTA->PCR[MOTOR_CONTORL_LEFT_backward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_LEFT_backward] |= PORT_PCR_MUX(1);
	PTA->PDDR |= (MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
	PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
	 
	// initialize rear leds
	PORTB->PCR[rear_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[rear_LED] |= PORT_PCR_MUX(1);
	
	// initialize front leds (PortE 20-23)
	PORTE->PCR[front_LED1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[front_LED1] |= PORT_PCR_MUX(1);
	PORTE->PCR[front_LED2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[front_LED2] |= PORT_PCR_MUX(1);
	PORTE->PCR[front_LED3] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[front_LED3] |= PORT_PCR_MUX(1);
	PORTE->PCR[front_LED4] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[front_LED4] |= PORT_PCR_MUX(1);
	
	// initialize front leds (PortB 0-3)
	PORTB->PCR[front_LED5] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[front_LED5] |= PORT_PCR_MUX(1);
	PORTB->PCR[front_LED6] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[front_LED6] |= PORT_PCR_MUX(1);
	PORTB->PCR[front_LED7] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[front_LED7] |= PORT_PCR_MUX(1);
	PORTB->PCR[front_LED8] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[front_LED8] |= PORT_PCR_MUX(1);
	
	// initialize front blinking leds (PortC 1)
	PORTC->PCR[front_blinking_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[front_blinking_LED] |= PORT_PCR_MUX(1);
	
	PTB->PDDR |= MASK(rear_LED);
	
	PTC->PDDR |= MASK(front_blinking_LED);
	
	
	PTE->PDDR |= MASK(front_LED1);
	PTE->PDDR |= MASK(front_LED2);
	PTE->PDDR |= MASK(front_LED3);
	PTE->PDDR |= MASK(front_LED4);
	
	PTB->PDDR |= MASK(front_LED5);
	PTB->PDDR |= MASK(front_LED6);
	PTB->PDDR |= MASK(front_LED7);
	PTB->PDDR |= MASK(front_LED8);
	 
 }
 
 
 void UART2_IRQHandler(void){
	 NVIC_ClearPendingIRQ(UART2_IRQn);
	 rx_data = UART2->D;
	 if(UART2->S1 & UART_S1_RDRF_MASK){
		 rx_data = UART2->D;
	 }
 }
 
 //sound part
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}
static void delay100x(volatile uint32_t nof) {
	for(int i =0;i<100;i++) {
		delay(nof);
	}
}
void playRunningMusic( void* argument){
	while(1){
// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
	int notes = sizeof(melody) / sizeof(melody[0]);
	// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
	int wholenote = (60000 * 4) / tempo;
	int divider = 0, noteDuration = 0;
	uint32_t period;
	while(1) {
		for(int i = 0; i<notes; i+=2) {
		// calculates the duration of each note
		divider = melody[i + 1];
		if (divider > 0) {
			// regular note, just proceed
			noteDuration = (wholenote) / divider;
		} else if (divider < 0) {
		// dotted notes are represented with negative durations!!
		noteDuration = (wholenote) / (int)fabs((float)divider);
		noteDuration *= 1.5; // increases the duration in half for dotted notes
		}
		period = TO_MOD(melody[i]);
		TPM0->MOD = period;
		TPM0_C0V = period / 8; //12.5% duty cycle
		delay100x(2*9*noteDuration);
		TPM0->MOD = 0;
		TPM0_C0V = 0;
		delay100x(10*noteDuration);
		}
	}
	}

}
 
 	void forward(void* argument){
			while (1){
				osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
				TPM2_C1V = 7000;
				TPM2_C0V = 7000;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
			}
			
	}
	void backward(void* argument){
		while (1){
			osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
			TPM2_C1V = 500;
			TPM2_C0V = 500;
			PTA->PDOR |= (MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
		}
	}
	
	void turnRight(void* argument){
		while (1){
			osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
			TPM2_C1V = 500;
			TPM2_C0V = 7000;
			PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
			PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
		}
		
		
	}
	
	void turnLeft(void* argument){
		while (1){
			osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
			TPM2_C1V = 7000;
			TPM2_C0V = 500;
			PTA->PDOR &= ~MASK(MOTOR_CONTORL_RIGHT_backward);
			PTA->PDOR |= MASK(MOTOR_CONTORL_LEFT_backward);
		}
		
		
	}
	
	void stop(void* argument){
			while (1){
			osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
			TPM2_C1V = 0;
			TPM2_C0V = 0;
			PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));	
			}
		
	}
	
	
 void led_off (void) {
	PTB->PDOR &= ~MASK(rear_LED);
	
	PTC->PDOR &= ~MASK(front_blinking_LED);
	
	PTE->PDOR &= ~MASK(front_LED1);
	PTE->PDOR &= ~MASK(front_LED2);
	PTE->PDOR &= ~MASK(front_LED3);
	PTE->PDOR &= ~MASK(front_LED4);
	
	PTB->PDOR &= ~MASK(front_LED5);
	PTB->PDOR &= ~MASK(front_LED6);
	PTB->PDOR &= ~MASK(front_LED7);
	PTB->PDOR &= ~MASK(front_LED8);
}


void rear_LED_moving (void *argument) {
	for(;;) {
	  osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		PTB->PDOR |= MASK(rear_LED);
		osDelay(500);
		PTB->PDOR &= ~MASK(rear_LED);
		osDelay(500);
		
	}

}

void rear_LED_stopped (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		led_off();
		PTB->PDOR |= MASK(rear_LED);
		osDelay(250);
		PTB->PDOR &= ~MASK(rear_LED);
		osDelay(250);
		
	}

}

void front_LED_moving (void *argument) {
	for(;;) {
	  osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
		PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
		
		PTE->PDOR |= MASK(front_LED1);
		PTE->PDOR &= (~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
		osDelay(500);
		
		PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED3) & ~MASK(front_LED4));
		PTE->PDOR |= MASK(front_LED2);
		osDelay(500);
		
		PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED4));
		PTE->PDOR |= MASK(front_LED3) ;
		osDelay(500);

		PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) );
		PTE->PDOR |= MASK(front_LED4);
		osDelay(500);
		
		PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
		PTB->PDOR &= ( ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
		PTB->PDOR |= MASK(front_LED5);
		osDelay(500);
		
		PTB->PDOR &= (~MASK(front_LED5)  & ~MASK(front_LED7) & ~MASK(front_LED8));
		PTB->PDOR |= MASK(front_LED6);
		osDelay(500);
		
		PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6)  & ~MASK(front_LED8));
		PTB->PDOR |= MASK(front_LED7);
		osDelay(500);
		
		PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) );
		PTB->PDOR |=  MASK(front_LED8);
		osDelay(500);

	}

}

void front_LED_stopped (void *argument) {
	for(;;) {
	  osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		PTB->PDOR |= (MASK(front_LED5) | MASK(front_LED6) | MASK(front_LED7) | MASK(front_LED8));
		PTE->PDOR |= (MASK(front_LED1) | MASK(front_LED2) | MASK(front_LED3) | MASK(front_LED4));
		

	}

}



void front_LED_blinking(void *argument) {
	for(;;) {
		
	  osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		PTB->PDOR &= ~(MASK(front_LED5) | MASK(front_LED6) | MASK(front_LED7) | MASK(front_LED8));
		PTE->PDOR &= ~(MASK(front_LED1) | MASK(front_LED2) | MASK(front_LED3) | MASK(front_LED4));
		
		PTC->PDOR |= MASK(front_blinking_LED);
		osDelay(500);
		PTC->PDOR &= ~MASK(front_blinking_LED);
		osDelay(500);
		
	}
}

 void getMove(void *argument){
	 	while(1){
			if (rx_data == 1){
				osThreadFlagsSet(front_LED_blinking_id,0x0001);			
			}
			if(rx_data == 2){
				//PTD->PDOR &= ~MASK (BLUE_LED);
				osThreadFlagsSet(forward_id,0x0001);
				osThreadFlagsSet(front_LED_moving_id,0x0001);
				osThreadFlagsSet(rear_LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 3){
				osThreadFlagsSet(backward_id,0x0001);
				osThreadFlagsSet(front_LED_moving_id,0x0001);
				osThreadFlagsSet(rear_LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 4){
				osThreadFlagsSet(left_id,0x0001);
				osThreadFlagsSet(front_LED_moving_id,0x0001);
				osThreadFlagsSet(rear_LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 5){
				osThreadFlagsSet(right_id,0x0001);
				osThreadFlagsSet(front_LED_moving_id,0x0001);
				osThreadFlagsSet(rear_LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 6){
				//PTD->PDOR |= MASK (BLUE_LED);
				osThreadFlagsSet(stop_id,0x0001);
				osThreadFlagsSet(front_LED_stopped_id,0x0001);
				osThreadFlagsSet(rear_LED_stopped_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
		}
}
 



int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPI0();
	initPWM();
	initUART2(BAUD_RATE);
	led_off();
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	//myMutex = osMutexNew(NULL);
  osThreadNew(getMove, NULL, NULL);    // Create application main thread
	forward_id = osThreadNew(forward,NULL,NULL);
	backward_id = osThreadNew(backward,NULL,NULL);
	left_id = osThreadNew(turnLeft,NULL,NULL);
	right_id = osThreadNew(turnRight,NULL,NULL);
	stop_id = osThreadNew(stop,NULL,NULL);
	
	rear_LED_moving_id = osThreadNew(rear_LED_moving, NULL, NULL);
  rear_LED_stopped_id = osThreadNew(rear_LED_stopped, NULL, NULL);
  front_LED_moving_id = osThreadNew(front_LED_moving, NULL, NULL);
  front_LED_stopped_id = osThreadNew(front_LED_stopped, NULL, NULL);
	front_LED_blinking_id = osThreadNew(front_LED_blinking, NULL, NULL);
	running_buzzer_id = osThreadNew(playRunningMusic,NULL, NULL);
	
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
