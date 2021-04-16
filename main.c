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

//self-drivering
#define trigger 12 //ptc12
#define echo 13 //ptc13

int volatile counter;
int volatile wait_delay;
int volatile duration = 10000;

//sound
#define buzzer 12
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

int melody2[]={
	DEGREE_E7,16, DEGREE_DR7,16, DEGREE_E7,16, DEGREE_DR7,16, DEGREE_E7,16, DEGREE_B6,16, DEGREE_D7,16, DEGREE_C7,16, DEGREE_A6,8,
};

int melody3[]={
	DEGREE_A5,8, DEGREE_B5,8,	//L1 1
	DEGREE_C6,-4, DEGREE_B5,8, DEGREE_C6,4, DEGREE_E6,4,	//2
	DEGREE_B5,-2, DEGREE_E5,4,	//3
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//4
	DEGREE_G5,-2, DEGREE_E5,4,	//5
	DEGREE_F5,-4, DEGREE_E5,8, DEGREE_F5,8, DEGREE_C6,-4,	//6
	DEGREE_E5,-2, DEGREE_C6,4,	//L2 7
	DEGREE_B5,-4, DEGREE_FR5,8, DEGREE_FR5,4, DEGREE_B5,4,	//8
	DEGREE_B5,2, REST,4, DEGREE_A5,8, DEGREE_B5,8,	//9
	DEGREE_C6,-4, DEGREE_C5,8, DEGREE_C6,4, DEGREE_E6,8,	//10
	DEGREE_B5,-2, DEGREE_E5,8, DEGREE_E5,8,	//11
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//L3 12
	DEGREE_G5,-2, DEGREE_E5,4,	//13
	DEGREE_F5,4, DEGREE_C6,8, DEGREE_B5,-4, DEGREE_C6,4,	//14
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_C6,2, REST,8,	//15
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_A5,4, DEGREE_B5,4, DEGREE_GR5,4,	//16
	DEGREE_A5,2, REST,4, DEGREE_C6,8, DEGREE_D6,8,	//L4 17
	DEGREE_E6,-4, DEGREE_C6,6, DEGREE_D6,6, DEGREE_E6,6,	//18
	DEGREE_D6,-2, DEGREE_G5,4,	//19
	DEGREE_D6,16, DEGREE_C6,16, DEGREE_B5,16, DEGREE_C6,-4, DEGREE_D6,4, DEGREE_E6,8,	//20
	DEGREE_E6,1,	//L5 21
	DEGREE_A5,8, DEGREE_B5,8, DEGREE_C6,4, DEGREE_B5,8, DEGREE_C6,8, DEGREE_D6,4,	//22
	DEGREE_C6,-4, DEGREE_G5,8, DEGREE_G5,2,	//23
	DEGREE_F6,4, DEGREE_E6,4, DEGREE_D6,4, DEGREE_C6,4,	//24
	DEGREE_E6,-2, DEGREE_A5,8, DEGREE_B5,8,	//25
	DEGREE_C6,-4, DEGREE_B5,8, DEGREE_C6,4, DEGREE_E6,4,	//26
	DEGREE_B5,-2, DEGREE_E5,4,	//L6 27
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//28
	DEGREE_G5,-2, DEGREE_E5,4,	//29
	DEGREE_F4,4, DEGREE_C6,8, DEGREE_B5,-4, DEGREE_C6,4,	//30
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_C6,2,	//31
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_A5,4, DEGREE_B5,4, DEGREE_GR5,4,	//32
	//Repeat once
	DEGREE_A5,8, DEGREE_B5,8,	//L1 1
	DEGREE_C6,-4, DEGREE_B5,8, DEGREE_C6,4, DEGREE_E6,4,	//2
	DEGREE_B5,-2, DEGREE_E5,4,	//3
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//4
	DEGREE_G5,-2, DEGREE_E5,4,	//5
	DEGREE_F5,-4, DEGREE_E5,8, DEGREE_F5,8, DEGREE_C6,-4,	//6
	DEGREE_E5,-2, DEGREE_C6,4,	//L2 7
	DEGREE_B5,-4, DEGREE_FR5,8, DEGREE_FR5,4, DEGREE_B5,4,	//8
	DEGREE_B5,2, REST,4, DEGREE_A5,8, DEGREE_B5,8,	//9
	DEGREE_C6,-4, DEGREE_C5,8, DEGREE_C6,4, DEGREE_E6,8,	//10
	DEGREE_B5,-2, DEGREE_E5,8, DEGREE_E5,8,	//11
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//L3 12
	DEGREE_G5,-2, DEGREE_E5,4,	//13
	DEGREE_F5,4, DEGREE_C6,8, DEGREE_B5,-4, DEGREE_C6,4,	//14
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_C6,2, REST,8,	//15
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_A5,4, DEGREE_B5,4, DEGREE_GR5,4,	//16
	DEGREE_A5,2, REST,4, DEGREE_C6,8, DEGREE_D6,8,	//L4 17
	DEGREE_E6,-4, DEGREE_C6,6, DEGREE_D6,6, DEGREE_E6,6,	//18
	DEGREE_D6,-2, DEGREE_G5,4,	//19
	DEGREE_D6,16, DEGREE_C6,16, DEGREE_B5,16, DEGREE_C6,-4, DEGREE_D6,4, DEGREE_E6,8,	//20
	DEGREE_E6,1,	//L5 21
	DEGREE_A5,8, DEGREE_B5,8, DEGREE_C6,4, DEGREE_B5,8, DEGREE_C6,8, DEGREE_D6,4,	//22
	DEGREE_C6,-4, DEGREE_G5,8, DEGREE_G5,2,	//23
	DEGREE_F6,4, DEGREE_E6,4, DEGREE_D6,4, DEGREE_C6,4,	//24
	DEGREE_E6,-2, DEGREE_A5,8, DEGREE_B5,8,	//25
	DEGREE_C6,-4, DEGREE_B5,8, DEGREE_C6,4, DEGREE_E6,4,	//26
	DEGREE_B5,-2, DEGREE_E5,4,	//L6 27
	DEGREE_A5,-4, DEGREE_G5,8, DEGREE_A5,4, DEGREE_C6,4,	//28
	DEGREE_G5,-2, DEGREE_E5,4,	//29
	DEGREE_F4,4, DEGREE_C6,8, DEGREE_B5,-4, DEGREE_C6,4,	//30
	DEGREE_D6,4, DEGREE_E6,8, DEGREE_C6,2,	//31
	DEGREE_C6,8, DEGREE_B5,8, DEGREE_A5,4, DEGREE_B5,4, DEGREE_GR5,4,	//32
	DEGREE_A5,1,	//33
};
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 osSemaphoreId_t mySem;
 uint8_t volatile rx_data=10 ;
 osThreadId_t  forward_id, backward_id, left_id, right_id, stop_id,self_driving_id,  
							 LED_moving_id, LED_stopped_id, connect_buzzer_id, end_buzzer_id,
							front_LED_blinking_id, running_buzzer_id, control_id;
 int current_note1 = 0;
 int current_note2 = 0;
 int current_note3 = 0;
 int current_front_led = 1;
 int rear_led_status = 1;

 
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
	
	PORTA->PCR[buzzer] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[buzzer] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	

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
 
 
 void init_pit(){
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[trigger] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[trigger] |= PORT_PCR_MUX(1);
	PORTC->PCR[echo] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[echo] |= PORT_PCR_MUX(1);
	
	PTC->PDDR |= MASK(trigger);
	PTC->PDDR &= ~MASK(echo);
	
	
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT->MCR = 0;
	
	PIT->CHANNEL[0].LDVAL = 47;
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
	
	NVIC_SetPriority(PIT_IRQn,2);
	NVIC_ClearPendingIRQ(PIT_IRQn);
	NVIC_EnableIRQ(PIT_IRQn);
}

void PIT_IRQHandler(){
	if(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK){
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;
		counter--;
		if(counter == 0){
			PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
			wait_delay = 0;
		}
	}
}

int detect(void){
	PTC->PDOR &= ~MASK(trigger);
	wait_delay = 1;
	counter = 2;
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	while(wait_delay);
	
	PTC->PDOR |= MASK(trigger);
	wait_delay = 1;
	counter = 10;
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	while(wait_delay);
	
	PTC->PDOR &= ~MASK(trigger);
	
	wait_delay = 1;
	counter = 10000;
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	
	while ((PTC->PDIR & MASK(echo)) == 0){
		if(!wait_delay){
			duration = 200000;
			return -1;
		}
	}
	while ((PTC->PDIR & MASK(echo)) == MASK(echo)){
		if(!wait_delay){
			duration = 200000;
			return -1;
		}
	}
	
	duration = 10000 - counter;
	
	if(duration <=400)
		return 1;
	else return 0;

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

	while(1) {
	osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
	int notes = sizeof(melody) / sizeof(melody[0]);
	// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
	int wholenote = (4000 * 4) / tempo;
	int divider = 0, noteDuration = 0;
	uint32_t period;
		if (current_note1 <= notes){
		// calculates the duration of each note
		divider = melody[current_note1 + 1];
		if (divider > 0) {
			// regular note, just proceed
			noteDuration = (wholenote) / divider;
		} else if (divider < 0) {
		// dotted notes are represented with negative durations!!
		noteDuration = (wholenote) / (int)fabs((float)divider);
		noteDuration *= 1.5; // increases the duration in half for dotted notes
		}
		period = TO_MOD(melody[current_note1]);
		TPM1->MOD = period;
		TPM1_C0V = period / 8; //12.5% duty cycle
		osDelay(2*9*noteDuration);
		TPM1->MOD = 0;
		TPM1_C0V = 0;
		osDelay(10*noteDuration);
		current_note1 += 2;
		}
		else current_note1 = 0;
	}

}

void playConnectMusic( void* argument){


	while(1) {
	osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
	int notes = sizeof(melody2) / sizeof(melody2[0]);
	// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
	int wholenote = (4000 * 4) / tempo;
	int divider = 0, noteDuration = 0;
	uint32_t period;
		if (current_note2 <= notes){
		// calculates the duration of each note
		divider = melody2[current_note2 + 1];
		if (divider > 0) {
			// regular note, just proceed
			noteDuration = (wholenote) / divider;
		} else if (divider < 0) {
		// dotted notes are represented with negative durations!!
		noteDuration = (wholenote) / (int)fabs((float)divider);
		noteDuration *= 1.5; // increases the duration in half for dotted notes
		}
		period = TO_MOD(melody2[current_note2]);
		TPM1->MOD = period;
		TPM1_C0V = period / 8; //12.5% duty cycle
		osDelay(2*9*noteDuration);
		TPM1->MOD = 0;
		TPM1_C0V = 0;
		osDelay(10*noteDuration);
		current_note2 += 2;
		}
		else current_note2 = 0;
	}

}

void playEndMusic( void* argument){
	while(1) {
	osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
	int notes = sizeof(melody3) / sizeof(melody3[0]);
	// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
	int wholenote = (4000 * 4) / tempo;
	int divider = 0, noteDuration = 0;
	uint32_t period;
		if (current_note3 <= notes){
		// calculates the duration of each note
		divider = melody3[current_note3 + 1];
		if (divider > 0) {
			// regular note, just proceed
			noteDuration = (wholenote) / divider;
		} else if (divider < 0) {
		// dotted notes are represented with negative durations!!
		noteDuration = (wholenote) / (int)fabs((float)divider);
		noteDuration *= 1.5; // increases the duration in half for dotted notes
		}
		period = TO_MOD(melody3[current_note3]);
		TPM1->MOD = period;
		TPM1_C0V = period / 8; //12.5% duty cycle
		osDelay(2*9*noteDuration);
		TPM1->MOD = 0;
		TPM1_C0V = 0;
		osDelay(10*noteDuration);
		current_note3 += 2;
		}
		else current_note3 = 0;
	}

}


 
 	void forward(void* argument){
			while (1){
				osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
				TPM2_C1V = 7000;
				TPM2_C0V = 7000;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				osDelay(100);

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
	
	




void LED_stopped (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		if(rear_led_status == 1){
			PTB->PDOR |= MASK(rear_LED);
			rear_led_status = 0;
			osDelay(250);
		}
		else if (rear_led_status == 0){
			PTB->PDOR &= ~MASK(rear_LED);
			rear_led_status = 1;
			osDelay(250);
		}
		PTB->PDOR |= (MASK(front_LED5) | MASK(front_LED6) | MASK(front_LED7) | MASK(front_LED8));
		PTE->PDOR |= (MASK(front_LED1) | MASK(front_LED2) | MASK(front_LED3) | MASK(front_LED4));
		
	}

}

void LED_moving (void *argument) {
	for(;;) {
	  osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
		
		if(current_front_led == 1){
			PTE->PDOR |= MASK(front_LED1);
			PTE->PDOR &= (~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTB->PDOR |= MASK(rear_LED);
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 2){
			PTB->PDOR &= ~MASK(rear_LED);
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTE->PDOR |= MASK(front_LED2);
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 3){
			PTB->PDOR |= MASK(rear_LED);
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED4));
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTE->PDOR |= MASK(front_LED3) ;
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 4){
			PTB->PDOR &= ~MASK(rear_LED);
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) );
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTE->PDOR |= MASK(front_LED4);
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 5){
			PTB->PDOR |= MASK(rear_LED);
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR &= ( ~MASK(front_LED6) & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTB->PDOR |= MASK(front_LED5);
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 6){
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR &= ~MASK(rear_LED);
			PTB->PDOR &= (~MASK(front_LED5)  & ~MASK(front_LED7) & ~MASK(front_LED8));
			PTB->PDOR |= MASK(front_LED6);
			current_front_led ++;
			osDelay(500);
		}
		else if (current_front_led == 7){
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR |= MASK(rear_LED);
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6)  & ~MASK(front_LED8));
			PTB->PDOR |= MASK(front_LED7);
			current_front_led ++;
			osDelay(500);
		}
		
		else if (current_front_led == 8){
			PTE->PDOR &= (~MASK(front_LED1) & ~MASK(front_LED2) & ~MASK(front_LED3) & ~MASK(front_LED4));
			PTB->PDOR &= ~MASK(rear_LED);
			PTB->PDOR &= (~MASK(front_LED5) & ~MASK(front_LED6) & ~MASK(front_LED7) );
			PTB->PDOR |=  MASK(front_LED8);
			current_front_led =1;
			osDelay(500);
		}

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
 


 void getMove(void *argument){
	 	for(;;){
			
			if (rx_data == 1){
				
				osThreadFlagsSet(front_LED_blinking_id,0x0001);
				osThreadFlagsSet(connect_buzzer_id,0x0001);
			}
			if(rx_data == 2){
				TPM2_C1V = 7000;
				TPM2_C0V = 7000;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 3){
				TPM2_C1V = 500;
				TPM2_C0V = 500;
				PTA->PDOR |= (MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 4){
				TPM2_C1V = 3000;
				TPM2_C0V = 4500;
				PTA->PDOR &= ~MASK(MOTOR_CONTORL_RIGHT_backward);
				PTA->PDOR |= MASK(MOTOR_CONTORL_LEFT_backward);
				
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 5){
				TPM2_C1V = 4500;
				TPM2_C0V = 3000;
				PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
				PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
				
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 6){
				TPM2_C1V = 0;
				TPM2_C0V = 0;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));	
				
				osThreadFlagsSet(LED_stopped_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 7){
				TPM2_C1V = 0;
				TPM2_C0V = 0;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));	
				led_off();
				osThreadFlagsSet(end_buzzer_id, 0x0001);
			}
			else if (rx_data == 8){
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
				
				TPM2_C1V = 7000;
				TPM2_C0V = 7000;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				osDelay(100);
				
				if(detect()){
					
					TPM2_C1V = 7000;
					TPM2_C0V = 500;
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR |= MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(450);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(500);
					
					TPM2_C1V = 500;
					TPM2_C0V = 7000;
					PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(500);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(1000);
					
					TPM2_C1V = 500;
					TPM2_C0V = 7000;
					PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(500);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(1000);
					
					TPM2_C1V = 500;
					TPM2_C0V = 7000;
					PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(500);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(1000);
					
					TPM2_C1V = 500;
					TPM2_C0V = 7000;
					PTA->PDOR |= MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(500);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(550);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 500;
					PTA->PDOR &= ~MASK(MOTOR_CONTORL_RIGHT_backward);
					PTA->PDOR |= MASK(MOTOR_CONTORL_LEFT_backward);
					osDelay(500);
					
					TPM2_C1V = 7000;
					TPM2_C0V = 7000;
					PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
					osDelay(2700);
					rx_data = 7;
				}
			}
			else if (rx_data == 0){
				TPM2_C1V = 7000;
				TPM2_C0V = 1200;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				
				osThreadFlagsSet(LED_moving_id,0x0001);
				osThreadFlagsSet(running_buzzer_id, 0x0001);
			}
			else if (rx_data == 9){
				TPM2_C1V = 1200;
				TPM2_C0V = 7000;
				PTA->PDOR &= ~(MASK(MOTOR_CONTORL_LEFT_backward) | MASK(MOTOR_CONTORL_RIGHT_backward));
				
				osThreadFlagsSet(LED_moving_id,0x0001);
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
	init_pit();
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	
  control_id = osThreadNew(getMove, NULL, NULL);   
	forward_id = osThreadNew(forward,NULL,NULL);
	//backward_id = osThreadNew(backward,NULL,NULL);
	//left_id = osThreadNew(turnLeft,NULL,NULL);
	//right_id = osThreadNew(turnRight,NULL,NULL);
	//stop_id = osThreadNew(stop,NULL,NULL);
	
	front_LED_blinking_id = osThreadNew(front_LED_blinking, NULL, NULL);
  LED_stopped_id = osThreadNew(LED_stopped, NULL, NULL);
  LED_moving_id = osThreadNew(LED_moving, NULL, NULL);
	running_buzzer_id = osThreadNew(playRunningMusic,NULL, NULL);
	connect_buzzer_id = osThreadNew(playConnectMusic,NULL, NULL);
	end_buzzer_id = osThreadNew(playEndMusic,NULL, NULL);
	//self_driving_id = osThreadNew(selfDriving,NULL,NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
