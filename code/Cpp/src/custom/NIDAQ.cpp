#include "../../include/custom/NIDAQ.h"
#include <cstdio>
#include <cstdlib>
#include <Windows.h>
#include <vector>  // For safe buffer management

// Macro for DAQmx error handling
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

//static function declaration
static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
                                        uInt32 nSamples, void *callbackData);

static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status,
                                      void *callbackData);


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
    return EXIT_SUCCESS;

Error:
    if (DAQmxFailed(error)) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("Digital Write Error: %s\n", errBuff);
    }
    return EXIT_FAILURE;
}

int ReadDigitalState(TaskHandle taskHandle) {
    int32 error = 0;
    int32 read = 0;
    uInt32 currentState = 0;

    DAQmxErrChk(DAQmxReadDigitalU32(
        taskHandle,
        1, // numSampsPerChan
        10.0, // timeout
        DAQmx_Val_GroupByChannel,
        &currentState,
        1, // arraySizeInSamps
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

int setLEDState(TaskHandle taskHandle, LED led, int state) {
    auto currentState = static_cast<uInt32>(ReadDigitalState(taskHandle));
    if (currentState == static_cast<uInt32>(-1)) return EXIT_FAILURE;
    if (state != LED_ON && state != LED_OFF) {
        printf("ERR: invalid LED state passed to setLEDState");
        return EXIT_FAILURE;
    }
    if (state == LED_ON) {
        currentState |= (1 << static_cast<int>(led));
    } else {
        currentState &= ~(1 << static_cast<int>(led));
    }
    if (WriteDigitalState(taskHandle, currentState) == -1) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// int setLEDOn(TaskHandle taskHandle, LED led) {
//     uInt32 currentState = static_cast<uInt32>(ReadDigitalState(taskHandle));
//     if (currentState == static_cast<uInt32>(-1)) return -1;
//
//     currentState |= (1 << static_cast<int>(led));
//     if (WriteDigitalState(taskHandle, currentState) == -1) {
//         return EXIT_FAILURE;
//     }
//     return EXIT_SUCCESS;
// }
//
// int setLEDOff(TaskHandle taskHandle, LED led) {
//     auto currentState = static_cast<uInt32>(ReadDigitalState(taskHandle));
//     if (currentState == static_cast<uInt32>(-1)) return -1;
//
//     currentState &= ~(1 << static_cast<int>(led));
//     if (WriteDigitalState(taskHandle, currentState) == -1) {
//         return EXIT_FAILURE;
//     }
//     return EXIT_SUCCESS;
// }

int setAllLEDs(const TaskHandle taskHandle, const int state) {
    if (state != LED_ON && state != LED_OFF) {
        printf("ERR: invalid LED state passed to setAllLEDs");
        return EXIT_FAILURE;
    }
    if (state == LED_ON) {
        return WriteDigitalState(taskHandle, 0x7);
    }
    return WriteDigitalState(taskHandle, 0x0);
}

// int setAllLEDsOff(TaskHandle taskHandle) {
//     return WriteDigitalState(taskHandle, 0x0);
// }

TaskHandle initDigitalOutputTask(const DigitalConfig &config) {
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

void cleanupDigitalOutputTask(const TaskHandle taskHandle) {
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
    uInt32 numChannels = 1;
};

void *DAQ_Init(const AnalogConfig &config, const DataCallback dataCB, const ErrorCallback errorCB) {
    int32 error = 0;
    auto *ctx = new AnalogContext();
    ctx->dataCallback = dataCB;
    ctx->errorCallback = errorCB;
    ctx->samplesPerCallback = config.samplesPerCallback;

    // Create task and channel
    DAQmxErrChk(DAQmxCreateTask("", &ctx->taskHandle));
    char chanSpec[100];
    snprintf(chanSpec, sizeof(chanSpec), "%s/%s", config.device, config.channel);
    DAQmxErrChk(DAQmxCreateAIVoltageChan(ctx->taskHandle, chanSpec, "",
        DAQmx_Val_Diff, config.minVoltage,
        config.maxVoltage, DAQmx_Val_Volts, nullptr));

    // Query number of channels
    DAQmxErrChk(DAQmxGetTaskNumChans(ctx->taskHandle, &ctx->numChannels));

    // Configure timing and callbacks
    DAQmxErrChk(DAQmxCfgSampClkTiming(ctx->taskHandle, "", config.sampleRate,
        DAQmx_Val_Rising, DAQmx_Val_ContSamps,
        config.samplesPerCallback * 10));
    DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(ctx->taskHandle,
        DAQmx_Val_Acquired_Into_Buffer, config.samplesPerCallback, 0,
        EveryNCallback, ctx));
    DAQmxErrChk(DAQmxRegisterDoneEvent(ctx->taskHandle, 0, DoneCallback, ctx));
    spdlog::info("DAQ_Init successful");
    return ctx;

Error:
    spdlog::error("DAQ_Init failed");
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

int DAQStart(void *handle) {
    const auto *ctx = static_cast<AnalogContext *>(handle);
    if (!ctx || !ctx->taskHandle) {
        // printf("ERR: incorrect handle passed to DAQ_Start\n");
        spdlog::error("Invalid handle passed to DAQ_Start");
        return EXIT_FAILURE;
    }

    if (DAQmxStartTask(ctx->taskHandle)) {
        if (ctx->errorCallback) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
            ctx->errorCallback(errBuff);
        }
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int DAQStop(void *handle) {
    const auto *ctx = static_cast<AnalogContext *>(handle);
    if (!ctx || !ctx->taskHandle) {
        // printf("ERR: invalid taskHandle passed to DAQ_Stop\n");
        spdlog::error("Invalid taskHandle passed to DAQ_Stop");
        return EXIT_FAILURE;
    }
    DAQmxStopTask(ctx->taskHandle);
    return EXIT_SUCCESS;
}

void DAQCleanup(void *handle) {
    const auto *ctx = static_cast<AnalogContext *>(handle);
    if (!ctx) return;

    if (ctx->taskHandle) {
        DAQmxStopTask(ctx->taskHandle);
        DAQmxClearTask(ctx->taskHandle);
    }
    delete ctx;
}

static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
                                        uInt32 nSamples, void *callbackData) {
    const auto *ctx = static_cast<AnalogContext *>(callbackData);

    int32 error = 0;
    int32 read = 0;

    // Use vector for safe memory management
    std::vector<float64> data((ctx->samplesPerCallback * ctx->numChannels));

    error = DAQmxReadAnalogF64(taskHandle,
                               ctx->samplesPerCallback,
                               10.0,
                               DAQmx_Val_GroupByScanNumber,
                               data.data(),
                               static_cast<int32>(ctx->samplesPerCallback * ctx->numChannels),
                               &read,
                               nullptr);

    if (DAQmxFailed(error)) {
        if (ctx->errorCallback) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
            ctx->errorCallback(errBuff);
        }
        return error;
    }

    if (read > 0 && ctx->dataCallback && static_cast<uInt32>(read) <= ctx->samplesPerCallback) {
        ctx->dataCallback(data.data(), static_cast<uInt32>(read));
    }

    return error;
}

static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status,
                                      void *callbackData) {
    const auto *ctx = static_cast<AnalogContext *>(callbackData);
    if (status < 0 && ctx->errorCallback) {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
        ctx->errorCallback(errBuff);
    }
    return 0;
}

// -------------------- Unified Initialization/Cleanup --------------------
DAQSystem initDAQSystem(const DigitalConfig &digiConfig,
                        const AnalogConfig &analogConfig,
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
        spdlog::error("DAQ Digital channel init fail");
        // printf("Digital initialization failed\n");
    }

    // Initialize analog input
    system.analogHandle = DAQ_Init(analogConfig, dataCB, errorCB);
    if (system.analogHandle) {
        analogOK = true;
    } else {
        // printf("Analog initialization failed\n");
        spdlog::error("DAQ Analog channel init fail");
    }

    // Set initialization flag
    system.initialized = (digitalOK && analogOK);

    // Cleanup if partial initialization occurred
    if (!system.initialized) {
        if (digitalOK) cleanupDigitalOutputTask(system.digitalTask);
        if (analogOK) DAQCleanup(system.analogHandle);
        system.digitalTask = nullptr;
        system.analogHandle = nullptr;
    }
    return system;
}

void cleanupDAQSystem(DAQSystem &system) {
    // Cleanup digital if initialized
    if (system.digitalTask) {
        cleanupDigitalOutputTask(system.digitalTask);
        system.digitalTask = nullptr;
    }

    // Cleanup analog if initialized
    if (system.analogHandle) {
        DAQStop(system.analogHandle);
        DAQCleanup(system.analogHandle);
        system.analogHandle = nullptr;
    }

    // Reset initialization flag
    system.initialized = false;
}
