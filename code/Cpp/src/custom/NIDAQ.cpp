#include "../../include/custom/NIDAQ.h"
#include <cstdio>
#include <cstdlib>
#include <Windows.h>
#include <vector>  // For safe buffer management

// Macro for DAQmx error handling
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// -------------------- Digital Output Implementation --------------------
int WriteDigitalState(TaskHandle taskHandle, uInt32 state) {
    int32 error = 0;
    int32 written = 0;

    DAQmxErrChk(DAQmxWriteDigitalU32(
        taskHandle,
        1, // numSampsPerChan
        1, // autoStart
        10.0, // timeout
        DAQmx_Val_GroupByChannel,
        &state, // writeArray
        &written,
        nullptr
    ));
    return 0;

Error:
    if (DAQmxFailed(error)) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("Digital Write Error: %s\n", errBuff);
    }
    return -1;
}

int ReadDigitalState(TaskHandle taskHandle) {
    int32 error = 0;
    int32 read = 0;
    uInt32 currentState = 0;

    DAQmxErrChk(DAQmxReadDigitalU32(
        taskHandle,
        1,                  // numSampsPerChan
        10.0,               // timeout
        DAQmx_Val_GroupByChannel,
        &currentState,
        1,                  // arraySizeInSamps
        &read,
        nullptr
    ));
    return static_cast<int>(currentState);

Error:
    if (DAQmxFailed(error)) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("Digital Read Error: %s\n", errBuff);
    }
    return -1;
}

int setLEDOn(TaskHandle taskHandle, LED led) {
    uInt32 currentState = static_cast<uInt32>(ReadDigitalState(taskHandle));
    if (currentState == static_cast<uInt32>(-1)) return -1;

    currentState |= (1 << static_cast<int>(led));
    if (WriteDigitalState(taskHandle, currentState) == -1) {
        return -1;
    }
    return 0;
}

int setLEDOff(TaskHandle taskHandle, LED led) {
    uInt32 currentState = static_cast<uInt32>(ReadDigitalState(taskHandle));
    if (currentState == static_cast<uInt32>(-1)) return -1;

    currentState &= ~(1 << static_cast<int>(led));
    if (WriteDigitalState(taskHandle, currentState) == -1) {
        return -1;
    }
    return 0;
}

int setAllLEDsOn(TaskHandle taskHandle) {
    return WriteDigitalState(taskHandle, 0x7);
}

int setAllLEDsOff(TaskHandle taskHandle) {
    return WriteDigitalState(taskHandle, 0x0);
}

TaskHandle initDigitalOutputTask(const DigitalConfig& config) {
    int32 error = 0;
    TaskHandle taskHandle = nullptr;

    DAQmxErrChk(DAQmxCreateTask("", &taskHandle));

    // Create channel specification
    char chanSpec[100];
    snprintf(chanSpec, sizeof(chanSpec), "%s/%s", config.device, config.channel);

    DAQmxErrChk(DAQmxCreateDOChan(taskHandle, chanSpec, "", DAQmx_Val_ChanForAllLines));
    DAQmxErrChk(DAQmxStartTask(taskHandle));

    return taskHandle;

Error:
    if (DAQmxFailed(error)) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("Digital Output Init Error: %s\n", errBuff);
    }
    if (taskHandle != nullptr) {
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
    return nullptr;
}

void cleanupDigitalOutputTask(TaskHandle taskHandle) {
    if (taskHandle != nullptr) {
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
}

// -------------------- Analog Acquisition Implementation --------------------
struct AnalogContext {
    TaskHandle taskHandle = nullptr;
    DataCallback dataCallback = nullptr;
    ErrorCallback errorCallback = nullptr;
    uInt32 samplesPerCallback = 0;
};

static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
                                        uInt32 nSamples, void* callbackData);
static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status,
                                      void* callbackData);

void* DAQ_Init(const AnalogConfig& config, DataCallback dataCB, ErrorCallback errorCB) {
    int32 error = 0;
    AnalogContext* ctx = new AnalogContext();
    ctx->dataCallback = dataCB;
    ctx->errorCallback = errorCB;
    ctx->samplesPerCallback = config.samplesPerCallback;

    // Create task and channel
    DAQmxErrChk(DAQmxCreateTask("", &ctx->taskHandle));
    char chanSpec[100];
    snprintf(chanSpec, sizeof(chanSpec), "%s/%s", config.device, config.channel);
    DAQmxErrChk(DAQmxCreateAIVoltageChan(ctx->taskHandle, chanSpec, "",
                                       DAQmx_Val_Cfg_Default, config.minVoltage,
                                       config.maxVoltage, DAQmx_Val_Volts, nullptr));

    // Configure timing and callbacks
    DAQmxErrChk(DAQmxCfgSampClkTiming(ctx->taskHandle, "", config.sampleRate,
                                    DAQmx_Val_Rising, DAQmx_Val_ContSamps,
                                    config.samplesPerCallback * 10));
    DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(ctx->taskHandle,
                DAQmx_Val_Acquired_Into_Buffer, config.samplesPerCallback, 0,
                EveryNCallback, ctx));
    DAQmxErrChk(DAQmxRegisterDoneEvent(ctx->taskHandle, 0, DoneCallback, ctx));

    return static_cast<void*>(ctx);

Error:
    if (error && errorCB) {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
        errorCB(errBuff);
    }
    if (ctx) {
        if (ctx->taskHandle) DAQmxClearTask(ctx->taskHandle);
        delete ctx;
    }
    return nullptr;
}

int DAQ_Start(void* handle) {
    AnalogContext* ctx = reinterpret_cast<AnalogContext*>(handle);
    if (!ctx) return -1;

    int32 error = DAQmxStartTask(ctx->taskHandle);
    if (error) {
        if (ctx->errorCallback) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
            ctx->errorCallback(errBuff);
        }
        return -1;
    }
    return 0;
}

int DAQ_Stop(void* handle) {
    AnalogContext* ctx = reinterpret_cast<AnalogContext*>(handle);
    if (!ctx || !ctx->taskHandle) return -1;
    DAQmxStopTask(ctx->taskHandle);
    return 0;
}

void DAQ_Cleanup(void* handle) {
    AnalogContext* ctx = reinterpret_cast<AnalogContext*>(handle);
    if (!ctx) return;

    if (ctx->taskHandle) {
        DAQmxStopTask(ctx->taskHandle);
        DAQmxClearTask(ctx->taskHandle);
    }
    delete ctx;
}

static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
                                        uInt32 nSamples, void* callbackData) {
    AnalogContext* ctx = reinterpret_cast<AnalogContext*>(callbackData);
    int32 error = 0;
    int32 read = 0;

    // Use vector for safe memory management
    std::vector<float64> data(ctx->samplesPerCallback);

    error = DAQmxReadAnalogF64(taskHandle, ctx->samplesPerCallback, 10.0,
                             DAQmx_Val_GroupByScanNumber, data.data(),
                             ctx->samplesPerCallback, &read, nullptr);

    if (DAQmxFailed(error)) {
        if (ctx->errorCallback) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
            ctx->errorCallback(errBuff);
        }
        return error;
    }

    if (read > 0 && ctx->dataCallback) {
        ctx->dataCallback(data.data(), static_cast<uInt32>(read));
    }

    return error;
}

static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status,
                                      void* callbackData) {
    AnalogContext* ctx = reinterpret_cast<AnalogContext*>(callbackData);
    if (status < 0 && ctx->errorCallback) {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
        ctx->errorCallback(errBuff);
    }
    return 0;
}

// -------------------- Unified Initialization/Cleanup --------------------
DAQSystem initDAQSystem(const DigitalConfig& digiConfig,
                        const AnalogConfig& analogConfig,
                        DataCallback dataCB,
                        ErrorCallback errorCB) {
    DAQSystem system;
    bool digitalOK = false;
    bool analogOK = false;

    // Initialize digital output
    system.digitalTask = initDigitalOutputTask(digiConfig);
    if (system.digitalTask) {
        digitalOK = true;
    } else {
        printf("Digital initialization failed\n");
    }

    // Initialize analog input
    system.analogHandle = DAQ_Init(analogConfig, dataCB, errorCB);
    if (system.analogHandle) {
        analogOK = true;
    } else {
        printf("Analog initialization failed\n");
    }

    // Set initialization flag
    system.initialized = (digitalOK && analogOK);

    // Cleanup if partial initialization occurred
    if (!system.initialized) {
        if (digitalOK) cleanupDigitalOutputTask(system.digitalTask);
        if (analogOK) DAQ_Cleanup(system.analogHandle);
        system.digitalTask = nullptr;
        system.analogHandle = nullptr;
    }

    return system;
}

void cleanupDAQSystem(DAQSystem& system) {
    // Cleanup digital if initialized
    if (system.digitalTask) {
        cleanupDigitalOutputTask(system.digitalTask);
        system.digitalTask = nullptr;
    }

    // Cleanup analog if initialized
    if (system.analogHandle) {
        DAQ_Stop(system.analogHandle);
        DAQ_Cleanup(system.analogHandle);
        system.analogHandle = nullptr;
    }

    // Reset initialization flag
    system.initialized = false;
}
