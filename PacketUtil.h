#include "define.h"

enum {
	PACKET_BUFF_MAX_SIZE = 25, //1 header + 10 * 2 ch ADS data + 3ch Accelerometer data + 1ch acb voltage data 
        NUMBER_OF_CHANNELS = 6, //2ch ADS, 3ch accelerometer, 1ch battery
        MAX_DIV = 10
};

uchar packetAddNewData(long* newData);//data size = 2 ch ADS1292 + 4ch ADC10. Returns 1 if buffer is full
uchar assemblePacket(); //returns packet size (number of bytes)
extern long* packet_buf;
extern uchar div[NUMBER_OF_CHANNELS];
extern uchar loffStatEnable;//0 - disable, 1 - enable
extern uchar loffStat;//loff stat value;
extern uint batteryVoltage;//ADC10 battery value
extern uchar analogInEnabled;
//------digital out----------
extern uchar dOutEnabled;
extern uint dOutPeriod; //disabled if dOutPeriod = 0
extern uint dOutLength; //Duty ratio
extern uint dOutNumOfPeriods; //number of periods. 0 - infinite; 
void stopDigitalOutGenerator(uchar outValue);
//-------------
void setAccelerometerMode(uchar mode);//0 - disable, 1 - 1channel mode, 3 - 3 channel mode;
void measureBatteryVoltage(uchar mode);
void packetUtilResetCounters();
 
