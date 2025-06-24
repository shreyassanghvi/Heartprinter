/*********************************************************************
*
* ANSI C Example program:
*    ContAcq-IntClk.c
*
* Description:
*    Continuously acquires analog voltage data from a DAQ device
*    using the internal clock. Data is read in chunks from the
*    device buffer and printed to the console.
*
*********************************************************************/

#include <cstdio>
#include <cstdlib>
#include "../../include/NIDAQmx/NIDAQmx.h"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// Callback function prototypes
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

int main_mainDAQ(int argc, char *argv[])
{
    int32       error = 0;
    TaskHandle  taskHandle = nullptr;
    char        errBuff[2048] = {'\0'};

    /*********************************************/
    // DAQmx Configure Code
    /*********************************************/
    DAQmxErrChk (DAQmxCreateTask("", &taskHandle));
    DAQmxErrChk (DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai0", "", DAQmx_Val_Cfg_Default, 0.0, 10.0, DAQmx_Val_Volts, nullptr));
    DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandle, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000));
    DAQmxErrChk (DAQmxRegisterEveryNSamplesEvent(taskHandle, DAQmx_Val_Acquired_Into_Buffer, 1000, 0, EveryNCallback, nullptr));
    DAQmxErrChk (DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, nullptr));

    /*********************************************/
    // DAQmx Start Code
    /*********************************************/
    DAQmxErrChk (DAQmxStartTask(taskHandle));

    printf("Acquiring samples continuously. Press Enter to interrupt\n");
    getchar();

Error:
    if( DAQmxFailed(error) )
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
    if( taskHandle != nullptr ) {
        /*********************************************/
        // DAQmx Stop Code
        /*********************************************/
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
    }
    if( DAQmxFailed(error) )
        printf("DAQmx Error: %s\n", errBuff);
    printf("End of program, press Enter key to quit\n");
    getchar();
    return 0;
}

/*****************************************************
* Callback: Called every 1000 samples acquired
*****************************************************/
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
    int32       error = 0;
    char        errBuff[2048] = {'\0'};
    static int  totalRead = 0;
    int32       read = 0;
    float64     data[1000];

    /*********************************************/
    // DAQmx Read Code
    /*********************************************/
    DAQmxErrChk (DAQmxReadAnalogF64(taskHandle, 1000, 10.0, DAQmx_Val_GroupByScanNumber, data, 1000, &read, NULL));
    if( read > 0 ) {
        printf("Acquired %d samples. Total %d\n", (int)read, (int)(totalRead += read));
        // Example: Print first 5 samples
        printf("First 5 samples: ");
        for(int i = 0; i < 5 && i < read; ++i)
            printf("%.3f ", data[i]);
        printf("\n");
        fflush(stdout);
    }

Error:
    if( DAQmxFailed(error) ) {
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        /*********************************************/
        // DAQmx Stop Code
        /*********************************************/
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
        printf("DAQmx Error: %s\n", errBuff);
    }
    return 0;
}

/*****************************************************
* Callback: Called when acquisition is done
*****************************************************/
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
    int32   error = 0;
    char    errBuff[2048] = {'\0'};

    // Check to see if an error stopped the task.
    DAQmxErrChk (status);

Error:
    if( DAQmxFailed(error) ) {
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        DAQmxClearTask(taskHandle);
        printf("DAQmx Error: %s\n", errBuff);
    }
    return 0;
}
