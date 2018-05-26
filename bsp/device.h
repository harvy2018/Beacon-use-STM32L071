#ifndef _DEVICE_H
#define _DEVICE_H

#include "stm32l0xx.h"


enum Button
{
    NEXT=0,
    OK,
    BACK
};

void RFSwitchON(void);
void RFSwitchOFF(void);

void BTSwitchON(void);
void BTSwitchOFF(void);
void DeviceInfoShow(u8 *ButtonState);
void BUZZSwitchON(void);
void BUZZSwitchOFF(void);
void ClearData(u8 *ButtonState);
void ScreenTest(u8 *ButtonState);
void Record_Review_B(u8 *ButtonState);
#endif