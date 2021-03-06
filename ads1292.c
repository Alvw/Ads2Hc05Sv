#include <msp430.h>
//#include "io430.h"
#include "ads1292.h"

uchar spiRxBuf[9];
uchar* AFE_CharBuff;
//uchar reg_map_debug[10];
const uchar regValues[] = {0x02, //reg 0x01  Set sampling ratio to 500 SPS
                          0xE3,  //reg 0x02 Set internal reference PDB_REFBUF = 1; int test enable
                          0x10,  //reg 0x03 
                          0x05,  //reg 0x04 Set Channel 1 to test
                          0x05,  //reg 0x05 Set Channel 2 to Input Short and disable
                          0x20,  //reg 0x06 Turn on Drl.
                          0x00,  //reg 0x07 
                          0x40,  //reg 0x08 clock divider Fclc/16 2048mHz external clock
                          0x02,  //reg 0x09 Set mandatory bit. RLD REF INT doesn't work without it.
                          0x03}; //reg 0x0A Set RLDREF_INT

/******************************************************************************/
/*                      ADS1292 initialization and start up sequence          */
/******************************************************************************/
void AFE_Init(){
  //configure connections to ADS1292
  //P4.4 - CS, P1.2 - DRDY, P4.5 - reset, P4.6 - start, P4.7 ClkSel
  //CS
  AFE_CS_DIR |= AFE_CS_PIN;
  AFE_CS_OUT |= AFE_CS_PIN; 
  //Reset
  AFE_RESET_DIR |= AFE_RESET_PIN;
  AFE_RESET_OUT |= AFE_RESET_PIN; 
  //Start
  AFE_START_DIR |= AFE_START_PIN;
  AFE_START_OUT &= ~AFE_START_PIN; //start pin low 
  //Clock select pin
//  AFE_CLOCK_SELECT_DIR |= AFE_CLOCK_SELECT_PIN; //Removed in new shcematic !!!!!!!!!!!!!!!!!!!!!!1111
//  AFE_CLOCK_SELECT_OUT &= ~AFE_CLOCK_SELECT_PIN; //0 - external clock //Removed in new shcematic !!!!!!!!!!!!!!!!!!!!!!1111
   
  //Spi setup
  UCB0CTL1 |= UCSWRST;                      // **Disable USCI state machine**
  UCB0CTL0 |= UCMST + UCMSB + UCSYNC;       // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_1;                     // ACLK !!!
  UCB0BR0 = 0x04;                           // UCLK/4 = 4 mHz
  UCB0BR1 = 0;
  //----------------------------------
  //P3REN = 0x00; // Pull-UP/DOWN Resistors Disabled
//----------------------------------------------
  
  TI_SPI_USCIB0_PxSEL |= TI_SPI_USCIB0_SIMO | TI_SPI_USCIB0_SOMI | TI_SPI_USCIB0_UCLK;              // SPI option select
  TI_SPI_USCIB0_PxDIR |= TI_SPI_USCIB0_SIMO | TI_SPI_USCIB0_UCLK;                      // SPI TXD out direction
  UCB0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
  
  //initialize DRDY pin (P1.2)
  P1REN |= AFE_DRDY_PIN; // Pull-UP/DOWN Resistors Enabled
  //P1IES &= ~AFE_DRDY_PIN;       // Interrupt on rising edge
  P1IES |= AFE_DRDY_PIN;       // Interrupt on falling edge
  P1IFG &= ~AFE_DRDY_PIN;      // Clear flag
  P1IE |= AFE_DRDY_PIN;        // Enable interrupt on DRDY
  
  //start up sequence for ADS1292
  __delay_cycles(16000000);                  //Wait after power-up until reset
  AFE_RESET_OUT &= ~AFE_RESET_PIN;
  __delay_cycles(320);                         //Reset low width
  AFE_RESET_OUT |= AFE_RESET_PIN;
  __delay_cycles(800);                         //delay before using ADS1292
  AFE_Cmd(0x11);                                 //stop continuous 
  AFE_Write_Reg(0x01,0x0A,regValues);
  AFE_Cmd(0x10);                         //start continuous 
  //AFE_START_OUT |= AFE_START_PIN;                           //start pin hi
}

void AFE_StartRecording(){
  AFE_START_OUT |= AFE_START_PIN;                           //start pin hi
}
void AFE_StopRecording(){
  AFE_START_OUT &= ~AFE_START_PIN;                           //start pin lo
}

/******************************************************************************/
/*                             ������� ������ �� SPI                          */
/* ���������:  1 ���� ������                                                  */
/* ����������: 1 ���� ������                                                  */
/******************************************************************************/
uchar AFE_SPI_Exchange (uchar tx_data)
{
  uchar rx_data;
  while (!(IFG2 & UCB0TXIFG)); // Wait for TXBUF ready
  UCB0TXBUF = tx_data;         // Send data
  while (UCB0STAT & UCBUSY);   // Wait for TX to complete  
  rx_data = UCB0RXBUF;         // Read received data
  return rx_data;
}

void spiReadData() {
  AFE_CS_OUT &= ~AFE_CS_PIN;                           
  uchar i = 0;
  for (; i < 9; i++) {
    spiRxBuf[i] = AFE_SPI_Exchange(0x00);
  }
  AFE_CS_DELAY;
  AFE_CS_OUT |= AFE_CS_PIN;                            
} 

void AFE_Cmd(uchar cmd) {
  AFE_CS_OUT &= ~AFE_CS_PIN;                           
  AFE_SPI_Exchange(cmd);
  AFE_CS_DELAY;
  AFE_CS_OUT |= AFE_CS_PIN;                            
}

void AFE_Write_Reg(uchar addr, uchar numOfBytes, const uchar* values) {
  AFE_CS_OUT &= ~AFE_CS_PIN; 
  AFE_SPI_Exchange(addr | 0x40);
  AFE_SPI_Exchange(numOfBytes);                      // Send number of bytes to write
  for(uchar i = 0; i< numOfBytes; i++){
    AFE_SPI_Exchange(values[i]);                     // Send data
  }               
  AFE_CS_DELAY;
  AFE_CS_OUT |= AFE_CS_PIN;                     
}

void AFE_Read_Reg(uchar addr, uchar numOfBytes, uchar* regBuf) {
  AFE_CS_OUT &= ~AFE_CS_PIN;                    // CS enable
  AFE_SPI_Exchange(addr | 0x20);                       // Send address
  AFE_SPI_Exchange(numOfBytes);                       // Send number bytes to read
  for(uchar i = 0; i < numOfBytes; i++){
    regBuf[i] = AFE_SPI_Exchange(0x00);
  }   
  AFE_CS_DELAY;
  AFE_CS_OUT |= AFE_CS_PIN;                     // CS disable
} 

void AFE_Read_Data(long* result){
  spiReadData();
  AFE_CharBuff = (uchar *) &result[0]; 
  AFE_CharBuff[3] = spiRxBuf[3];
  AFE_CharBuff[2] = spiRxBuf[4];
  AFE_CharBuff[1] = spiRxBuf[5];
  result[0] = result[0] >> 8;
  
  AFE_CharBuff = (uchar *) &result[1]; 
  AFE_CharBuff[3] = spiRxBuf[6];
  AFE_CharBuff[2] = spiRxBuf[7];
  AFE_CharBuff[1] = spiRxBuf[8];
  result[1] = result[1] >> 8;
}

uchar AFE_getLoffStatus(){
  uchar result = ((spiRxBuf[0]<<1)&0x0E) | ((spiRxBuf[1]>>7)&0x01);
  return result;
}

















