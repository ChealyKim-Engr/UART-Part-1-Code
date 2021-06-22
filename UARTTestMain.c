// UARTTestMain.c
// Runs on LM4F120/TM4C123
// Used to test the UART.c driver
// Daniel Valvano
// September 12, 2013

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

// Port F bit 4 is negative logic switch to play sound, SW1
// Port F bit 1 is negative logic switch to play sound, SW2

//NOTE:
#define PF4_SW1      (*((volatile unsigned long *)0x40025040))
#define PF3          (*((volatile unsigned long *)0x40025020))
#define PF2          (*((volatile unsigned long *)0x40025010))
#define PF1      		 (*((volatile unsigned long *)0x40025008))
#define PF0_SW2      (*((volatile unsigned long *)0x40025004))
//#define SW1       0x10                      // on the left side of the Launchpad board
//#define SW2       0x01                      // on the right side of the Launchpad board


#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#include "PLL.h"
#include "UART.h"
#include "tm4c123gh6pm.h"

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mod4
void GPIOPortF_Handler(void);
void PortF_Init(void);
void GPIOPortF_Handler(void);

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

////  red, green, blue
//const long ColorWheel[8] = {0x02,0x08,0x04};
	unsigned char interrupt;
int main(void){
	unsigned char led;
	unsigned long green, blue, red, ledValue;
	green = 0; blue = 0; red = 0;

	DisableInterrupts(); 
  PLL_Init();
  UART_Init();              // initialize UART
	PortF_Init();            // initialize buttons and LEDs on Port F
	EnableInterrupts();

  while(1){
		UART_OutString("Choose LED:  ");
		if ((interrupt != 1) && (interrupt != 2)) 
			led = UART_InChar();	
		else if (interrupt == 1) {
			led = '1';
		}
		else if (interrupt == 2) {
			led = '0';
		}
		UART_OutChar(led);		OutCRLF();		
		switch(led) {
			case 'g': {
				if (green == 8) {
					UART_OutString("Green is OFF!");	OutCRLF();
					GPIO_PORTF_DATA_R &= ~0x08;	
				}
				else if(green == 0){
					UART_OutString("Green is ON!");	OutCRLF();
					GPIO_PORTF_DATA_R |= 0x08;	
				}
			}
			break;
			case 'r': {
				if (red == 2) {
					UART_OutString("Red   is OFF!");	OutCRLF();
					GPIO_PORTF_DATA_R &= ~0x02;	
				}
				else if (red == 0){
					UART_OutString("Red   is ON!");	OutCRLF();
					GPIO_PORTF_DATA_R |= 0x02;	
				}
			}
			break;
			case 'b': {
				if (blue == 4) {
					UART_OutString("Blue   is OFF!");	OutCRLF();
					GPIO_PORTF_DATA_R &= ~0x04;	
				}
				else if (blue == 0){
					UART_OutString("Blue   is ON!");	OutCRLF();
					GPIO_PORTF_DATA_R |= 0x04;	
				}
			}
			break;
			case '1': {
				if (green == 0) {
					UART_OutString("Green is ON!");	
					OutCRLF();
				}
				if (blue  == 0) {
					UART_OutString("Blue  is ON!");	
					OutCRLF();
				}
				if (red   == 0) {
					UART_OutString("Red   is ON!"); 
					OutCRLF();
				}
				UART_OutString("ALL LED have been turned ON!");	OutCRLF();
				GPIO_PORTF_DATA_R |= 0x0E; 
				interrupt = 0;
			}
			break;
			case '0': {
				if (green == 8){
					UART_OutString("Green is OFF!");	
					OutCRLF();
				}
				if (blue  == 4) {
					UART_OutString("Blue  is OFF!");	
					OutCRLF();
				}
				if (red   == 2) {
					UART_OutString("Red   is OFF!");	
					OutCRLF();
				}				
				UART_OutString("ALL LED have been turned OFF!");	OutCRLF();
				GPIO_PORTF_DATA_R &= ~0x0E;
				interrupt = 0;
			}
			break;
			default: {
				led= led;
			}
			break;		
		}	
		red   = GPIO_PORTF_DATA_R&0x02;
		green = GPIO_PORTF_DATA_R&0x08;
		blue  = GPIO_PORTF_DATA_R&0x04;
		ledValue = GPIO_PORTF_DATA_R;
		UART_OutUDec(ledValue);   OutCRLF();
		OutCRLF();
  }
}

void PortF_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x20;         // 1) activate Port F
  delay = SYSCTL_RCGC2_R;         // allow time for clock to stabilize
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) unlock Port F lock
  GPIO_PORTF_CR_R = 0x1F;         //    enable commit (allow configuration changes) on PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;      // 3) disable analog functionality on PF4-0
  GPIO_PORTF_PCTL_R = 0x00000000; // 4) configure PF4-0 as GPIO
  GPIO_PORTF_DIR_R = 0x0E;        // 5) PF4 and PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;      // 6) disable alt funct on PF4-0
  GPIO_PORTF_DEN_R = 0x1F;        // 7) enable digital I/O on PF4-0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}
void GPIOPortF_Handler(void){ 
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		interrupt = 1;
  }
	else if(GPIO_PORTF_RIS_R&0x01){ // SW2 touch
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		interrupt = 2;
	}
	InterruptFlag(interrupt);
}
///// SW2 (send color) connected to PF0
////// Red LED connected to PF1
////// Blue LED connected to PF2
////// Green LED connected to PF3
////// SW1 (step color) connected to PF4
//  red, yellow, green, light blue, blue, purple,  white,  dark
//const long ColorWheel[8] = {0x02,0x0A,0x08,0x0C,0x04,0x06,0x0E,0x00};
