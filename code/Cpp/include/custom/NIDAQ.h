//
// Created by desktop on 6/24/2025.
//

#ifndef NIDAQ_CUSTOM_H
#define NIDAQ_CUSTOM_H
#include <conio.h>
#include <cstdio>
#include <cstdlib>
#include <Windows.h>
#include "../../include/NIDAQmx/NIDAQmx.h"
enum LED {
    LEFT_BASE,
    CENTER_BASE,
    RIGHT_BASE
};
int setAllLEDsOff(const TaskHandle taskHandle);
int setAllLEDsOn(const TaskHandle taskHandle);
int setLEDOn(const TaskHandle taskHandle, LED led);
int setLEDOff(const TaskHandle taskHandle, LED led);

TaskHandle initDigitalOutputTask();
void cleanupDigitalOutputTask(TaskHandle taskHandle);


#endif //NIDAQ_CUSTOM_H
