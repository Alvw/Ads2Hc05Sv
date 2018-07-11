#include <msp430.h>
//#include "io430.h"
#include "subroutine.h"
#include "rf.h"
#include "ads1292.h"
#include "ADC10.h"
#include "PacketUtil.h"

#define helloMsg "\xAA\xA5\x05\xA0\x55"
#define firmwareVersion "\xAA\xA5\x07\xA1\x02\x01\x55" //version 2.1 digital in, digital out
#define unknownMultiByteCmdError "\xAA\xA5\x07\xA2\x00\x00\x55"
#define unfinishedIncomingMsgError "\xAA\xA5\x07\xA2\x00\x01\x55"
#define unknownOneByteCmdError     "\xAA\xA5\x07\xA2\x00\x02\x55" 
#define noStopMarkerError          "\xAA\xA5\x07\xA2\x00\x03\x55"
#define txFailError          "\xAA\xA5\x07\xA2\x00\x04\x55"
#define lowBatteryMessage "\xAA\xA5\x07\xA3\x00\x01\x55"
#define hardwareConfigMessage "\xAA\xA5\x07\xA4\x02\x01\x55"//reserved, power button, 2ADS channels, 1 accelerometer
#define stopRecordingResponceMessage "\xAA\xA5\x05\xA5\x55"//reserved, power button, 8ADS channels, 1 accelerometer
#define chargeShutDownMessage "\xAA\xA5\x05\xA6\x55"
void onRF_MessageReceived();
void onRF_MultiByteMessage();
void startRecording();
void stopRecording();
uchar packetDataReady = 0;
uchar lowBatteryMessageAlreadySent = 0;
uchar shutDownCntr = 0;
uchar pingCntr = 0; 
uchar rfResetCntr = 0;
uint powerUpCntr = 1;
uchar btnCntr = 0;
uchar isRecording = 0;
uint sumBatteryVoltage = 8000;
uchar batteryCntr = 0;

//таймаут до перезагрузки RF модуля в количестве циклов таймера. 1 цикл таймера ~ 0.25 секунды.
// 0 - перезагрузка отключена
uint resetTimeout = 0; 

int main(void)
{
  __disable_interrupt();
  sys_init();
led(1);
  ADC10_Init();
  AFE_Init();
  rf_init();
   TACCR0 = 0xFFFF;// запуск таймера
  __enable_interrupt();

 while (1)
 {
   if(rf_rx_data_ready_fg) {
     onRF_MessageReceived();
     rf_rx_data_ready_fg = 0;
   }
   if (packetDataReady){       
     uchar packetSize = assemblePacket();
       rf_send((uchar*)&packet_buf[0], packetSize);
     packetDataReady = 0;      
   }
   if(rf_rx_data_ready_fg || packetDataReady){
  // идем по циклу снова
   }else{
   __bis_SR_register(CPUOFF + GIE); // Уходим в спящий режим 
   }
 }
} 
/*-----------------Обработка полученного с компьютера сообщения--------------*/
void onRF_MessageReceived(){
    switch(rf_rx_buf[0]){
    case 0xFF: //stop recording command
      stopRecording();
      rf_send(stopRecordingResponceMessage,5);
      break;
    case 0xFE: //start recording command
      startRecording();
      break;
    case 0xFD: //hello command
      rf_send(helloMsg,5);
      break;
    case 0xFC: //версия прошивки
      rf_send(firmwareVersion,7);
      break;
    case 0xFB: //ping command
      pingCntr = 0;
      break;
    case 0xFA: //hardware config request
        rf_send(hardwareConfigMessage,7);
      break;
    case 0xF9: //start digital out generator
        dOutEnabled = 1;
      break;
    case 0xF8: //stop digital out generator and set out to 1
        stopDigitalOutGenerator(1);
    case 0xF7: //stop digital out generator and set out to 0
        stopDigitalOutGenerator(0);
      break;
    default:
      if((rf_rx_buf_size < rf_rx_buf[0]) && (rf_rx_buf[0] < 0xFA)){//проверяем длину однобайтовой команды
        rf_send(unknownOneByteCmdError,7);
      }
      //проверяем два последних байта == маркер конца пакета
      if(((rf_rx_buf[rf_rx_buf[0]-1] == 0x55) && (rf_rx_buf[rf_rx_buf[0]-2] == 0x55))){
        onRF_MultiByteMessage();
      }else{
        rf_send(noStopMarkerError,7);
      }
      break;
    }
}

void onRF_MultiByteMessage(){
  uchar msgOffset = 1;
  while (msgOffset < (rf_rx_buf[0]-2)){
    if(rf_rx_buf[msgOffset] == 0xF0){//команда для ads1292
      AFE_Cmd(rf_rx_buf[msgOffset+1]);
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF1){//Запись регистров ads1292
      AFE_Write_Reg(rf_rx_buf[msgOffset+1], rf_rx_buf[msgOffset+2], &rf_rx_buf[msgOffset+3]);
      msgOffset+=rf_rx_buf[msgOffset+2]+3;
    }else if(rf_rx_buf[msgOffset] == 0xF2){//делители частоты для ads1292 2 значения
      for(int i = 0; i<2; i++){
        if((rf_rx_buf[msgOffset+1+i] == 0) || (rf_rx_buf[msgOffset+1+i] == 1) || 
           (rf_rx_buf[msgOffset+1+i] == 2) || (rf_rx_buf[msgOffset+1+i] == 5) || (rf_rx_buf[msgOffset+1+i] == 10)){
          div[i] = rf_rx_buf[msgOffset+1+i]; 
        }
      }
      msgOffset+=3;
    }else if(rf_rx_buf[msgOffset] == 0xF3){//Режим работы акселерометра
      setAccelerometerMode(rf_rx_buf[msgOffset+1]);
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF4){//Передача данных батарейки
      measureBatteryVoltage(rf_rx_buf[msgOffset+1]);
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF5){//передача данных loff статуса 
      loffStatEnable = rf_rx_buf[msgOffset+1];
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF6){//RF reset timeout при отсутствии Ping команды с компьютера. 
      resetTimeout = rf_rx_buf[msgOffset+1] * 4;
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF7){//передача данных аналогового входа AnalogIn 
      analogInEnabled = rf_rx_buf[msgOffset+1];
      msgOffset+=2;
    }else if(rf_rx_buf[msgOffset] == 0xF8){//включение аналогового выхода
      uchar* chBuf = (uchar*) &dOutPeriod;
      chBuf[0] = rf_rx_buf[msgOffset+1];
      chBuf[1] = rf_rx_buf[msgOffset+1];
      chBuf = (uchar*) &dOutLength;
      chBuf[0] = rf_rx_buf[msgOffset+1];
      chBuf[1] = rf_rx_buf[msgOffset+1];
      chBuf = (uchar*) &dOutNumOfPeriods;
      chBuf[0] = rf_rx_buf[msgOffset+1];
      chBuf[1] = rf_rx_buf[msgOffset+1];
      if( dOutPeriod){
        dOutEnabled = 1;
      }else{
        dOutEnabled = 0;
      }
      msgOffset+=7;
    }else if(rf_rx_buf[msgOffset] == 0xFE){//start recording command 
       startRecording();
       msgOffset+=1;
    }else{
      rf_send(unknownMultiByteCmdError,7);
      return;
    }
  }
}

void startRecording(){
      isRecording = 1;
       powerUpCntr = 0;
       packetUtilResetCounters();
       lowBatteryMessageAlreadySent = 0;
       shutDownCntr = 0;
       pingCntr = 0;
       AFE_StartRecording();
       led(0);
}

void stopRecording(){
  isRecording = 0; 
  powerUpCntr = 1;
  pingCntr = 0;
  AFE_StopRecording();
}

void addBatteryData(uint battValue){
  sumBatteryVoltage +=battValue;
  batteryCntr++;
  if(batteryCntr == 10){
    if(sumBatteryVoltage < BATT_LOW_TH){
        if(shutDownCntr == 0) {shutDownCntr = 1;}  
     }
    sumBatteryVoltage = 0;
    batteryCntr = 0;
  }
}

/* ------------------------ Прерывание от P1 ----------------------- */

#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
  if (P1IFG & AFE_DRDY_PIN) { 
    P1IFG &= ~AFE_DRDY_PIN;      // Clear DRDY flag
    long new_data[6];
    AFE_Read_Data(&new_data[0]);
    loffStat = AFE_getLoffStatus();
    ADC10_Read_Data(&new_data[2]);
    ADC10_Measure();
    if(packetAddNewData(new_data)){
      packetDataReady = 1;
      __bic_SR_register_on_exit(CPUOFF); // Не возвращаемся в сон при выходе
    }
  }
}
/* -------------------------------------------------------------------------- */
/* ------------------------- Прерывание от таймера -------------------------- */
/* -------------------------------------------------------------------------- */
#pragma vector = TIMERA1_VECTOR
__interrupt void TimerA_ISR(void)
{ 

TACTL &= ~TAIFG;
  if(rfResetCntr == 0x01){
    P3OUT |= BIT7;//BT reset pin hi
    rfResetCntr = 0;
  }
  if(isRecording){
    if(resetTimeout){
      pingCntr++;
      if(pingCntr > resetTimeout){//no signal from host for ~ resetTimeout * 4 seconds
        P3OUT &= ~BIT7; //BT reset pin lo
        if (rfResetCntr == 0) rfResetCntr = 1;
        pingCntr = 0;
      }
    }
  }else{//if not recording
    long new_data[4];
    ADC10_Read_Data(&new_data[0]); 
    addBatteryData(new_data[3]);
    ADC10_Measure();
  }
  if(BIT0 & P1IN){// if rf connected
      rfConStat = 1;
  }else{
      rfConStat = 0;
  }
  if(!lowBatteryMessageAlreadySent){    
      if(batteryVoltage < BATT_LOW_TH){
        lowBatteryMessageAlreadySent = 1;
        rf_send(lowBatteryMessage,7);
        if(shutDownCntr == 0) {shutDownCntr = 1;}  
      }
  }
  if(powerUpCntr){
    powerUpCntr++;
    if(powerUpCntr >= 2400){//забыли выключить питание (не стартует запись в теечение ~10 минут)
        if(shutDownCntr == 0) {shutDownCntr = 1;}
    }
    if(powerUpCntr%2){
      led(1);
    }else{
      led(0);
    }
  }
  if(shutDownCntr){
    shutDownCntr++;
    if(shutDownCntr > 4){//wait 1 second before shut down
      AFE_StopRecording();
      P3OUT |= BIT0; //accelerometer power down
      P3OUT &= ~BIT7;//BT reset pin low  
      P4OUT &= ~BIT7; //BT LDO power down
      P4OUT &= ~BIT5; //ADS reset/powerdown
      TACCR0 = 0x00;
      led(0);
      __bis_SR_register(LPM3_bits); // Уходим в спящий режим 
    }
  } 
  if(rf_delete_unfinished_incoming_messages()){
    rf_send(unfinishedIncomingMsgError,7);
  }
  if(rf_tx_fail_flag){//debug information maybe delete
    rf_tx_fail_flag = 0;
    rf_send(txFailError,7);
  }
}
/* -------------------------------------------------------------------------- */ 

