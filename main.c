/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MOTOR_CONTORL_RIGHT_forward 12 //PortA pin 12 TPM1_CH0
#define MOTOR_CONTORL_RIGHT_backward 13// PORTA pin 13 TPM1_CH1
#define MOTOR_CONTORL_LEFT_forward 1 //PortA pin 1 TPM2_CH0
#define MOTOR_CONTORL_LEFT_backward 3// PORTB pin 3 TPM2_CH1
#define LED_ON 'o'
#define LED_OFF 'i'
#define MASK(x) (1 << (x))
#define UART_RX_PORTE23 23 //uart receive
#define BAUD_RATE 9600
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 osSemaphoreId_t mySem;
 osMutexId_t myMutex;
 uint8_t volatile rx_data ;
 
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
	
	PORTA->PCR[MOTOR_CONTORL_RIGHT_backward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_RIGHT_backward] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[MOTOR_CONTORL_LEFT_forward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_LEFT_forward] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[MOTOR_CONTORL_LEFT_backward] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[MOTOR_CONTORL_LEFT_backward] |= PORT_PCR_MUX(3);
	
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
}



 void initGPI0(){
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
	 
	PTB->PDOR |= MASK(RED_LED) | MASK(GREEN_LED);
  PTD->PDOR |= MASK (BLUE_LED);
	 
 }
 
 static void delay(volatile uint32_t nof){
	 while(nof!=0){
		 __asm("NOP");
		 nof--;
		 
	 }
 }
 
 
 void UART2_IRQHandler(void){
	 NVIC_ClearPendingIRQ(UART2_IRQn);
	 rx_data = UART2->D;
	 if(UART2->S1 & UART_S1_RDRF_MASK){
		 rx_data = UART2->D;
	 }
	 //PORTE->ISFR = 0xffffffff;
 }
 
 	void forward(){
			TPM1_C0V = 7000;
			TPM1_C1V = 0;
			TPM2_C0V = 7000;
			TPM2_C1V = 0;
	}
	void backward(){
			TPM1_C0V = 0;
			TPM1_C1V = 7000;
			TPM2_C0V = 0;
			TPM2_C1V = 7000;
	}
	
	void turnRight(){
		TPM1_C1V = 7000;
		TPM2_C0V = 7000;
	}
	
	void turnLeft(){
		TPM1_C0V = 7000;
		TPM2_C1V = 7000;
	}
	
	void stop(){
		TPM1_C0V = 0;
		TPM1_C1V = 0;
		TPM2_C0V = 0;
		TPM2_C1V = 0;
	}
 
 void getMove(void *argument){
	 	while(1){
			if(rx_data == 2){
				forward();
			}
			else if (rx_data == 3){
				backward();
			}
			else if (rx_data == 4){
				turnLeft();
			}
			else if (rx_data == 5){
				turnRight();
			}
			else if (rx_data == 6){
				stop();
			}
		}
}
 

	

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPI0();
	initPWM();
	initUART2(BAUD_RATE);
	
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	myMutex = osMutexNew(NULL);
  osThreadNew(getMove, NULL, NULL);    // Create application main thread
	
	
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
