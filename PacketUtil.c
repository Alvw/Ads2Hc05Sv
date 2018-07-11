#include <msp430.h>
//#include "io430.h"
#include "PacketUtil.h"
#include "string.h"

 
long buf1[PACKET_BUFF_MAX_SIZE];
long buf2[PACKET_BUFF_MAX_SIZE];
uint analogIn1;
uint analogIn2;
uint packet_cntr = 0;
long* add_buf = &buf1[0];
long* packet_buf = &buf2[0];
uchar div[NUMBER_OF_CHANNELS] ={1,1,10,10,10,0}; // frequency dividers for each channel. div = 0 if channel is not active.
uchar offsetCounter[NUMBER_OF_CHANNELS] = {0,0,0,0,0,0}; // offsetCounter[i]_max = ( MAX_DIV / div[i] )  - number of "long" to buffer data from channel i
uchar sumCounter[NUMBER_OF_CHANNELS]= {0,0,0,0,0,0}; // sumCounter[i]_max = div[i] - how many times we sum input data from channel i to have average output data
uchar loffStatEnable = 0;//0 - disable, 1 - enable
uchar loffStat;
uint batteryVoltage = 8000;
//-------digital in-----
uint* analogInToWrite = &analogIn1;
uint* analogInToRead = &analogIn2;
uchar analogInEnabled = 0;
uint analogInBit = 0; 
//-----digital out------
uchar dOutEnabled = 0;
uint dOutPeriod = 0; //disabled if dOutPeriod = 0
uint dOutLength = 0; //Duty ratio
uint dOutNumOfPeriods = 0; //number of periods. 0 - infinite; 
uint dOutDrdyCounter = 0;
uint dOutPeriodsCounter = 0;
//-----------

void packetUtilResetCounters(){
  memset(offsetCounter, 0, NUMBER_OF_CHANNELS); 
  memset(sumCounter, 0, NUMBER_OF_CHANNELS); 
  packet_cntr = 0;
  dOutDrdyCounter = 0;
  dOutPeriodsCounter = 0;
}

void setAccelerometerMode(uchar mode){//0 - disable, 1 - enable;
  for(uchar i = 0; i<3; i++){
    if(mode){
      div[i+2] = 10;
    }else{
      div[i+2] = 0;
    }
  }
}

void measureBatteryVoltage(uchar mode){//0 - disable, 1 - enable;
    if(mode){
      div[5] = 10;
    }else{
      div[5] = 0;
    }
}

uchar packetAddNewData(long* newData) {
  uchar isAccumulatingFinished = 0;
  uchar j = 0;
  for (uchar i = 0; i < NUMBER_OF_CHANNELS; i++) {
    if (div[i] != 0) {
      if(sumCounter[i] == 0){
        add_buf[1 + j + offsetCounter[i]] = 0;
      }
      add_buf[1 + j + offsetCounter[i]] += newData[i];
      sumCounter[i]++;
      if (sumCounter[i] == div[i]) {
        offsetCounter[i]++;
        if ((sumCounter[i] * offsetCounter[i]) == MAX_DIV) {
          isAccumulatingFinished = 1;
          offsetCounter[i] = 0;
        }
        sumCounter[i] = 0;
      }
      j += (MAX_DIV / div[i]);      // j += offsetCounter[i]_max
    }
  }
  //-----------digital in-------------
  if(analogInEnabled){
      if(analogInBit == 0){
        analogInBit = 1;
      } else{
        analogInBit = analogInBit << 1;
      }
      if(BIT4 & P2IN) {   //Analog In p2.4
        *analogInToWrite |= analogInBit;
      } else {
        *analogInToWrite &= ~analogInBit;
      }  
    }
  //-------------end digital in-----------
  //---------digital out-------
    if(dOutEnabled){
      if(dOutDrdyCounter++ == 0){
        P4OUT |= BIT0;
      }
      if(dOutDrdyCounter == dOutLength){
        P4OUT &= ~BIT0;
      }
      if(dOutDrdyCounter == dOutPeriod){
        dOutDrdyCounter = 0;
        if(dOutPeriodsCounter++ == dOutNumOfPeriods){
          stopDigitalOutGenerator(0);
        }
      }
    }
    //---------end digital out---
  if(isAccumulatingFinished){
    //flip buffers
    long* pBuf = add_buf;
    add_buf = packet_buf;
    packet_buf = pBuf;
    //flip analogIn buffers
    uint* pAnIn = analogInToWrite;
    analogInToWrite = analogInToRead;
    analogInToRead = pAnIn;
    //reset analog in bit;
    analogInBit = 0;
  }
  return isAccumulatingFinished;
} 

void stopDigitalOutGenerator(uchar outValue){
          dOutEnabled = 0;
          dOutDrdyCounter = 0;
          dOutPeriodsCounter = 0;
          if(outValue){
            P4OUT |= BIT0;
          }else{
            P4OUT &= ~BIT0;
          }
}

uchar assemblePacket(){
  uint* intBuff = (uint *) &packet_buf[0];
  intBuff[0] = 0xAAAA;
  intBuff[1] = packet_cntr++; 
  uchar* packetCharBuff = (uchar *) &packet_buf[1];
  uchar charIndex = 0;
  uchar longIndex = 0;
  //Add ADS1292 data
  for (uchar i = 0; i < 2; i++) {// 2 channels
    if (div[i] != 0) {
      uchar numberOfSamplesInChannel = (MAX_DIV/div[i]);
      for(uchar j = 0; j < numberOfSamplesInChannel; j++){
        packet_buf[longIndex + 1]/= div[i];
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4];
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4 + 1];
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4 + 2];
        longIndex++;
      }
    }
  }
  //Add accelerometer data
  for (uchar i = 2; i < 5; i++) {// 3 channels
    if (div[i] != 0) {
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4];
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4 + 1];
        longIndex++;
    }
  }
  //Add battery data
  if (div[5] != 0) {
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4];
        packetCharBuff[charIndex++] = packetCharBuff[longIndex*4 + 1];
        batteryVoltage = *(uint *)&packetCharBuff[longIndex*4];
        longIndex++;
  }
  //Add loff status if enabled
  if(loffStatEnable){
    packetCharBuff[charIndex++] = loffStat;
  }
  
  //Add analog in value if enabled (2 bites)
  if(analogInEnabled){
    uchar* charAnalogIn = (uchar*)analogInToRead;
    packetCharBuff[charIndex++] = charAnalogIn[0];
    packetCharBuff[charIndex++] = charAnalogIn[1];
  }
  //Add footer value 0x55
  packetCharBuff[charIndex++] = 0x55;
  return 4 + charIndex ;
}














