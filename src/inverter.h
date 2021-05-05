#ifndef INVERTER_H
#define INVERTER_H

#include <Arduino.h>

#define INVERTER_COMMAND_TIMEOUT_MS 5000
#define INVERTER_COMMAND_DELAY_MS 500

//Send and receive periodic inverter commands
void serviceInverter();

struct QpiMessage
{
  byte protocolId;
};

struct P003PSMessage
{
  unsigned long rxTimeSec;
  float solarWatt1;
  float solarWatt2;
  float batteryWatt;
  float acin2_r;
  float acin2_s;
  float acin2_t;
  float acin2_total;
  float w_r;
  float w_s;
  float w_t;
  float w_total;
  float va_r;
  float va_s;
  float va_t;
  float va_total;
  float ac_output_procent;  
};

struct P006FPADJMessage
{  //-- '34'^D0301,0000,1,0099,1,0109,1,0112⸮7'
  unsigned long rxTimeSec;
  float dir;
  float watt;
  float feedingGridDirectionR;
  float calibrationWattR;  
  float feedingGridDirectionS;
  float calibrationWattS; 
  float feedingGridDirectionT;
  float calibrationWattT; 
};

struct P003GSMessage
{
  unsigned long rxTimeSec;
  float solarInputV1;
  float solarInputV2;
  float solarInputA1;
  float solarInputA2;
  float battV;
  float battCapacity;
  float battA;
  float acInputVoltageR;
  float acInputVoltageS;
  float acInputVoltageT;
  float acInputFrequency;
  float acInputCurrentR;
  float acInputCurrentS;
  float acInputCurrentT;
  float acOutputVoltageR;
  float acOutputVoltageS;
  float acOutputVoltageT;
  float acOutputFrequency;
  float acOutputCurrentR;
  float acOutputCurrentS;
  float acOutputCurrentT;
};



struct QpigsMessage
{
  unsigned long rxTimeSec;
  //087.6 51.18 08.81 04.71 04.10 0450 +034 00.05 -030 0000 11000000O⸮'
  float solarV;
  float battV;
  float battChargeA;
  float solarA;
  float solar2A;
  float wattage;
}; 

struct QmodMessage
{
  char mode;
};

struct QpiwsMessage
{
  bool reserved0;
  bool inverterFault;
  bool busOver;
  bool busUnder;
  bool busSoftFail;
  bool lineFail;
  bool opvShort;
  bool overTemperature;
  bool fanLocked;
  bool batteryVoltageHigh;
  bool batteryLowAlarm;
  bool reserved13;
  bool batteryUnderShutdown;
  bool reserved15;
  bool overload;
  bool eepromFault;
  bool inverterOverCurrent;
  bool inverterSoftFail;
  bool selfTestFail;
  bool opDcVoltageOver;
  bool batOpen;
  bool currentSensorFail;
  bool batteryShort;
  bool powerLimit;
  bool pvVoltageHigh;
  bool mpptOverloadFault;
  bool mpptOverloadWarning;
  bool batteryTooLowToCharge;
  bool reserved30;
  bool reserved31;
};

struct QflagMessage
{
  bool disableBuzzer;
  bool enableOverloadBypass;
  bool enablePowerSaving;
  bool enableLcdEscape;
  bool enableOverloadRestart;
  bool enableOvertempRestart;
  bool enableBacklight;
  bool enablePrimarySourceInterruptedAlarm;
  bool enableFaultCodeRecording;
};

struct QidMessage
{
  char id[16];
};

#endif
