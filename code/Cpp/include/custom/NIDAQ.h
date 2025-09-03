#pragma once

#include <conio.h>
#include <cstdio>
#include <cstdlib>
#include <Windows.h>
#include "../../include/NIDAQmx/NIDAQmx.h"

#include <spdlog/spdlog.h>
// -------------------- Type Definitions --------------------
#define LED_ON 1
#define LED_OFF 0

enum class LED : int {
    LEFT_BASE = 0,
    CENTER_BASE = 1,
    RIGHT_BASE = 2
};

// Callback types for analog acquisition
using DataCallback = void(*)(double *data, uInt32 numSamples);
using ErrorCallback = void(*)(const char *errorMessage);

// -------------------- Configuration Structures --------------------
struct DigitalConfig {
    const char *device; // e.g. "Dev1"
    const char *channel; // e.g. "PFI0:2"
};

struct AnalogConfig {
    const char *device; // e.g. "Dev1"
    const char *channel; // e.g. "ai0"
    double minVoltage; // e.g. 0.0
    double maxVoltage; // e.g. 10.0
    double sampleRate; // e.g. 10000.0
    uInt32 samplesPerCallback; // e.g. 1000
};

// -------------------- Unified DAQ System Handle --------------------
struct DAQSystem {
    TaskHandle digitalTask = nullptr;
    void *analogHandle = nullptr;
    bool initialized = false;
};

// -------------------- Function Declarations --------------------
// Digital LED Control
// int setLEDOn(TaskHandle taskHandle, LED led);
//
// int setLEDOff(TaskHandle taskHandle, LED led);
int setLEDState(TaskHandle taskHandle, LED led, int state);

int setAllLEDs(TaskHandle taskHandle, int state);

//int setAllLEDsOn(TaskHandle taskHandle);
// int setAllLEDsOff(TaskHandle taskHandle);

// Analog Acquisition Control
int DAQStart(void *handle);

int DAQStop(void *handle);


// Unified Initialization and Cleanup
DAQSystem initDAQSystem(const DigitalConfig &digiConfig,
                        const AnalogConfig &analogConfig,
                        DataCallback dataCB = nullptr,
                        ErrorCallback errorCB = nullptr);

void cleanupDAQSystem(DAQSystem &system);
