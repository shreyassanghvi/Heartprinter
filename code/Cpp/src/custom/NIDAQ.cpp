#include "../../include/custom/NIDAQ.h"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

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
    return 0xFFFF;
}


int ReadDigitalState(TaskHandle taskHandle) {
    int32 error = 0;
    int32 read = 0;
    uInt32 currentState = 0;
    // Read current state
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
    return read;
    Error:
        if (DAQmxFailed(error)) {
            char errBuff[2048] = {'\0'};
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("Digital Read Error: %s\n", errBuff);
        }
    return 0xFFFF;
}
int setLEDOn(const TaskHandle taskHandle, LED led) {

    uInt32 currentState = ReadDigitalState(taskHandle);
    if (currentState == 0xFFFF)
        return -1;
    currentState |= (1 << led);
    if (WriteDigitalState(taskHandle, currentState) == 0xFFFF) {
        return -1;
    }
    return 0;
}

int setLEDOff(const TaskHandle taskHandle, LED led) {

    uInt32 currentState = ReadDigitalState(taskHandle);
    if (currentState == 0xFFFF)
        return -1;
    currentState &= ~(1 << led);
    if (WriteDigitalState(taskHandle, currentState) == 0xFFFF) {
        return -1;
    }
    return 0;
}

int setAllLEDsOn(const TaskHandle taskHandle) {
    if (WriteDigitalState(taskHandle, 0x7) == 0xFFFF) {
        return -1;
    }
    return 0;
}

int setAllLEDsOff(const TaskHandle taskHandle) {
    if (WriteDigitalState(taskHandle, 0x0) == 0xFFFF) {
        return -1;
    }
    return 0;
}

// Initialize DAQ task and return handle
TaskHandle initDigitalOutputTask() {
    int32 error = 0;
    TaskHandle taskHandle = NULL;

    DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
    DAQmxErrChk(DAQmxCreateDOChan(taskHandle, "Dev1/PFI0:2", "", DAQmx_Val_ChanForAllLines));
    DAQmxErrChk(DAQmxStartTask(taskHandle));

    return taskHandle;

    Error:
        if (DAQmxFailed(error)) {
            char errBuff[2048] = {'\0'};
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("Initialization Error: %s\n", errBuff);
        }
    if (taskHandle != NULL) {
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
    return nullptr;
}

// Cleanup DAQ resources
void cleanupDigitalOutputTask(TaskHandle taskHandle) {
    if (taskHandle != NULL) {
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
}

//
// int main(int argc, char *argv[]) {
//     int32 error = 0;
//     TaskHandle taskHandle = NULL;
//     char errBuff[2048] = {'\0'};
//
//
//     /*********************************************/
//     // DAQmx Configuration
//     /*********************************************/
//     DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
//     DAQmxErrChk(DAQmxCreateDOChan(taskHandle, "Dev1/PFI0:2", "", DAQmx_Val_ChanForAllLines));
//
//     /*********************************************/
//     // Initialize all lines to high-impedance (OFF)
//     /*********************************************/
//     // DAQmxErrChk(DAQmxSetDOTristate(taskHandle, "Dev1/PFI0:5", TRUE));
//
//     /*********************************************/
//     // Start task and sequence LEDs
//     /*********************************************/
//     DAQmxErrChk(DAQmxStartTask(taskHandle));
//
//     printf("Sequencing LEDs. Press Enter to exit.\n");
//     while (!_kbhit()) {
//         // Loop until a key is pressed
//         for (int i = 0; i < 3; ++i) {
//             int32 written;
//             setAllLEDsOff(taskHandle);
//             Sleep(500); // Wait 500 ms before next state
//         }
//     }
//     getchar(); // Wait for Ente
//
// Error:
//     if (DAQmxFailed(error))
//         DAQmxGetExtendedErrorInfo(errBuff, 2048);
//     if (taskHandle != nullptr) {
//         DAQmxStopTask(taskHandle);
//         DAQmxClearTask(taskHandle);
//     }
//     if (DAQmxFailed(error))
//         printf("DAQmx Error: %s\n", errBuff);
//     printf("End of program, press Enter key to quit\n");
//     getchar();
//     return 0;
// }
