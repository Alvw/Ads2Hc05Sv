#include <msp430.h>
//#include "io430.h"
#include "subroutine.h"

//������������� ����������������
void sys_init(){
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
 // CLOCK
  BCSCTL1 = CALBC1_16MHZ;                    
  DCOCTL = CALDCO_16MHZ;
  BCSCTL2 |= DIVS_3;                        // SMCLK / 8;
  
  //SMCLC output pin
  P1DIR |= BIT4; //P1.4 = output direction
  P1SEL |= BIT4; //P1.4 = SMCLK output function
  
  BCSCTL1 |= XTS;                           // ACLK = LFXT1 = HF XTAL
  BCSCTL3 |= LFXT1S1;                       // 3 � 16MHz crystal or resonator
  IE1 |= OFIE;                              // Enable osc fault interrupt 
  
  //LED
  P1DIR |= BIT7;
  P1OUT &= ~BIT7;
  
//initialize BT_CON_STAT pin (P1.0)
  P1DIR &= ~BIT0; //input
  P1REN |= BIT0; // Pull-UP/DOWN Resistors Enabled
  P1OUT |= BIT0; //pull up resistor direction
//  P1IES &= ~BIT0;       // Interrupt on rising edge
//  P1IFG &= ~BIT0;      // Clear flag
//  P1IE |= BIT0;        // Enable interrupt on DRDY
  
//RF reset pin
  P3DIR |= BIT7;
  P3OUT &= ~BIT7;
  
//RF enable pin p 4.7
  P4DIR |= BIT7;
  P4OUT |= BIT7;
  
//P3.0 Accelerometer Power Down
  P3DIR |= BIT0;
  P3OUT &= ~BIT0;
  
//P4.0 Digital out
  P4DIR |= BIT0;
  P4OUT |= BIT0;
  
//P2.4 Digital in
  P2DIR &= ~BIT4; //input
  P2REN |= BIT4; //Pull up/down resistor enabled
  P2OUT |= BIT4; //pull up resistor direction
  
  
// �������������� ������
  P1DIR |= BIT1 + BIT3 + BIT5 + BIT6;
  P1OUT &= ~(BIT1 + BIT3 + BIT5 + BIT6);
  
  P2DIR |= BIT4 + BIT5;
  P2OUT &= ~(BIT4 + BIT5);
  
  P3DIR |= BIT0;
  P3OUT &= ~BIT0;
  
  P4DIR |= BIT0 + BIT1 + BIT2 + BIT3;
  P4OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3); 
  
  // ������ 
  TACTL |= TACLR;
  //TACTL_bit.TACLR  = 1; // Reset TAR, divider and count dir
  TACTL = TASSEL_2;     // SMCLK
  TACTL |= ID_2 + ID_1; // 1:8  
  TACCR0 = 0x00;
  TACTL |= TAIE;
 // TACTL_bit.TAIE = 1;   // INT enable 
  TACTL &= ~TAIFG;      // ����� ����������
  TACTL |= MC_1;
}

#pragma bis_nmi_ie1=OFIE                    // Re-enable osc fault interrupt
#pragma vector=NMI_VECTOR
__interrupt void NMI_ISR(void)
{
  volatile unsigned int i;
  BCSCTL2 &= ~SELS;                       // Ensure SMCLK runs from DCO 
  do {
    IFG1 &= ~OFIFG;                         // Clear OSCFault flag
    for (i = 0xFF; i > 0; i--);             // Time for flag to set
  } while (IFG1 & OFIFG);                     // OSCFault flag still set?
  BCSCTL2 |= SELS;                        // SMCLK = LFXT1 / 8;
} 

void led(uchar state){
  if(state){
    P1OUT |= BIT7;
  }else{
    P1OUT &=~BIT7;
  }
}

/* --------------------- ��������� "�������" -------------------- */
void Pwr_Indication()
{
  P1OUT &=~BIT7;
  for (unsigned char cntr = 0; cntr < 6; cntr++) // ������ 3 ����
     {
       P1OUT ^= BIT7;
       __delay_cycles(3200000);
     } 
}
/* -------------------------------------------------------------------------- */