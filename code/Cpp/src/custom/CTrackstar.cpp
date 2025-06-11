#include "../../include/custom/ATC3DG.h"
#include "../../include/custom/CTrackstar.h"

// for Trackstar


CSensor *pSensor; // a pointer to an array of sensor objects
CXmtr *pXmtr; // a pointer to an array of transmitter objects
CSystem ATC3DG; // a pointer to a single instance of the system class


/**
 * @brief Check if a key is pressed on the console.
 * @return 1 if a key is pressed, 0 if not.
 *
 * This function is used to check if a key is pressed on the console, and is
 * used by the example programs to check if the user wants to exit.
 *
 * Note: This function is not portable and may not work on all systems.
 */
int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
    termios oldt{}, newt{};

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// This is a simplified error handler.
// This error handler takes the error code and passes it to the GetErrorText()
// procedure along with a buffer to place an error message string.
// This error message string can then be output to a user display device
// like the console
// Specific error codes should be parsed depending on the application.
//
void errorHandler(int error, int lineNum) {
    char buffer[1024];
    int currentError = error;
    int nextError;

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //  The following code shows you how to use the procedure GetErrorText().
    //
    //  When making the call you will pass the errorcode to be decoded and
    //  a pointer to a buffer where the message string will be placed
    //  Note: This procedure like all the others will also return an error code.
    //  This new error code will either indicate a problem with the call itself or
    //  will simply be the next error code in the system error code queue.
    //  Looping on a test of this returned error code will cause the
    //  extraction of all current errors in the queue.
    //
    do {
        nextError = GetErrorText(currentError, buffer, sizeof(buffer), SIMPLE_MESSAGE);

        //////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////
        //
        //	Display the message string for the "current error"
        //
        //  Insert display mechanism of choice here. As an example this sample
        //	simply sends the message string to the console display using printf()
        //	Note: The message strings returned from the call do not contain a
        //	terminating newline character. If the user needs the strings to be
        //	displayed on succeeding lines then a newline "\n" needs to be added.
        //
        printf("%s %d\n", buffer, lineNum);

        currentError = nextError;
    } while (currentError != BIRD_ERROR_SUCCESS);

    exit(0);
}

void initTxRx() {
    int errorCode; // used to hold error code returned from procedure call
    int i;
    int sensorID;
    int transmitterID;
    short id;

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    printf("\n\n");
    printf("ATC3DG setup..\n");
    //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
    //
    // Initialize the ATC3DG driver and DLL
    //
    // It is always necessary to first initialize the ATC3DG "system". By
    // "system" we mean the set of ATC3DG cards installed in the PC. All cards
    // will be initialized by a single call to InitializeBIRDSystem(). This
    // call will first invoke a hardware reset of each board. If at any time
    // during operation of the system an unrecoverable error occurs then the
    // first course of action should be to attempt to Recall InitializeBIRDSystem()
    // if this doesn't restore normal operating conditions there is probably a
    // permanent failure - contact tech support.
    // A call to InitializeBIRDSystem() does not return any information.
    //
    printf("Initializing ATC3DG system...\n");
    errorCode = InitializeBIRDSystem();
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GET SYSTEM CONFIGURATION
    //
    // In order to get information about the system we have to make a call to
    // GetBIRDSystemConfiguration(). This call will fill a fixed size structure
    // containing amongst other things the number of boards detected and the
    // number of sensors and transmitters the system can support (Note: This
    // does not mean that all sensors and transmitters that can be supported
    // are physically attached)
    //
    errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // The SYSTEM_CONFIGURATION structure filled out by the initialization proc
    // contains the following:
    printf("Number Boards           = %d\n", ATC3DG.m_config.numberBoards);
    printf("Number Sensors          = %d\n", ATC3DG.m_config.numberSensors);
    printf("Number Transmitters     = %d\n\n", ATC3DG.m_config.numberTransmitters);

    printf("System AGC mode         = %d\n", ATC3DG.m_config.agcMode);
    printf("Maximum Range           = %6.2f\n", ATC3DG.m_config.maximumRange);
    printf("Measurement Rate        = %10.6f\n", ATC3DG.m_config.measurementRate);
    printf("Metric Mode             = %d\n", ATC3DG.m_config.metric);
    printf("Line Frequency          = %6.2f\n", ATC3DG.m_config.powerLineFrequency);
    printf("Transmitter ID Running  = %d\n", ATC3DG.m_config.transmitterIDRunning);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GET SENSOR CONFIGURATION
    //
    // Having determined how many sensors can be supported we can dynamically
    // allocate storage for the information about each sensor.
    // This information is acquired through a call to GetSensorConfiguration()
    // This call will fill a fixed size structure containing amongst other things
    // a status which indicates whether a physical sensor is attached to this
    // sensor port or not.
    //
    pSensor = new CSensor[ATC3DG.m_config.numberSensors];
    for (i = 0; i < ATC3DG.m_config.numberSensors; i++) {
        errorCode = GetSensorConfiguration(i, &pSensor[i].m_config);
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
    }

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GET TRANSMITTER CONFIGURATION
    //
    // The call to GetTransmitterConfiguration() performs a similar task to the
    // GetSensorConfiguration() call. It also returns a status in the filled
    // structure which indicates whether a transmitter is attached to this
    // port or not. In a single transmitter system it is only necessary to
    // find where that transmitter is in order to turn it on and use it.
    //
    pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
    for (i = 0; i < ATC3DG.m_config.numberTransmitters; i++) {
        errorCode = GetTransmitterConfiguration(i, &pXmtr[i].m_config);
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
    }
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GetSystemParameter()
    //
    // This procedure allows the user to inspect selected system variables. The
    // difference between these variables and sensor or transmitter variables  is
    // that a change to one of these variables will effect the entire system
    // The user must pass a parameter of enumerated type SYSTEM_PARAMETER_TYPE
    // together with a pointer to a buffer for the result
    // As with all procedures an error code is returned
    //
    printf("\nGetSystemParameter()");
    printf("\n====================\n");
    //
    // SELECT_TRANSMITTER
    //
    {
        short buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(SELECT_TRANSMITTER, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("SELECT_TRANSMITTER: %d\n", buffer);
    }
    //
    // POWER_LINE_FREQUENCY
    //
    {
        double buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(POWER_LINE_FREQUENCY, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("POWER_LINE_FREQUENCY: %5.2f\n", buffer);
    }
    //
    // AGC_MODE
    //
    {
        AGC_MODE_TYPE buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(AGC_MODE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("AGC_MODE: %d\n", buffer);
    }
    //
    // MEASUREMENT_RATE
    //
    {
        double buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(MEASUREMENT_RATE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("MEASUREMENT_RATE: %5.2f\n", buffer);
    }
    //
    // MAXIMUM_RANGE
    //
    {
        double buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(MAXIMUM_RANGE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("MAXIMUM_RANGE: %5.2f\n", buffer);
    }
    //
    // METRIC
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSystemParameter(METRIC, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("METRIC: %d\n", buffer);
    }

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //	The previous calls can simplified using the macros provided in SAMPLE2.H
    //  (printf statements are provided for clarity)
    //
    printf("\nSetSystemParameter()");
    printf("\n=============================\n");
    printf("SELECT_TRANSMITTER:		0\n");
    SET_SYSTEM_PARAMETER(SELECT_TRANSMITTER, 0, __LINE__);
    printf("POWER_LINE_FREQUENCY:   50.0\n");
    SET_SYSTEM_PARAMETER(POWER_LINE_FREQUENCY, 50.0, __LINE__);
    printf("AGC_MODE:               SENSOR_AGC_ONLY\n");
    SET_SYSTEM_PARAMETER(AGC_MODE, SENSOR_AGC_ONLY, __LINE__);
    printf("MEASUREMENT_RATE:       75.0\n");
    SET_SYSTEM_PARAMETER(MEASUREMENT_RATE, 75.0, __LINE__);
    printf("MAXIMUM_RANGE:          72.0\n");
    SET_SYSTEM_PARAMETER(MAXIMUM_RANGE, 72.0, __LINE__);
    printf("METRIC:                 true\n");
    SET_SYSTEM_PARAMETER(METRIC, true, __LINE__);


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GetSensorParameter()
    //
    // This procedure allows the user to inspect selected sensor parameters. The
    // procedure is directed at a specific sensor by selecting that sensor with
    // the Sensor ID parameter. For the purpose of this exercise the ID = 0. This
    // is the 1st sensor in a system. This sensor will need to be attached for
    // these calls to respond without an error.
    //
    printf("\nGetSensorParameter()");
    printf("\n====================\n");
    printf("Using sensor ID = 0\n");
    sensorID = 0;
    //
    // DATA_FORMAT
    //
    {
        DATA_FORMAT_TYPE buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, DATA_FORMAT, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("DATA_FORMAT: %d\n", buffer);
    }
    //
    // ANGLE_ALIGN
    //
    {
        DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, ANGLE_ALIGN, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("ANGLE_ALIGN: %6.2f, %6.2f, %6.2f\n",
               buffer.a,
               buffer.e,
               buffer.r);
    }
    //
    // HEMISPHERE
    //
    {
        HEMISPHERE_TYPE buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, HEMISPHERE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("HEMISPHERE: %4x\n", buffer);
    }
    //
    // FILTER_AC_WIDE_NOTCH
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_AC_WIDE_NOTCH, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_AC_WIDE_NOTCH: %d\n", buffer);
    }
    //
    // FILTER_AC_NARROW_NOTCH
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_AC_NARROW_NOTCH, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_AC_NARROW_NOTCH: %d\n", buffer);
    }
    //
    // FILTER_DC_ADAPTIVE
    //
    {
        double buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_DC_ADAPTIVE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_DC_ADAPTIVE: %5.2f\n", buffer);
    }
    //
    // FILTER_ALPHA_PARAMETERS
    //
    {
        ADAPTIVE_PARAMETERS buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_ALPHA_PARAMETERS, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_ALPHA_PARAMETERS:\n");
        printf("    Alpha max   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.alphaMax[0],
               buffer.alphaMax[1],
               buffer.alphaMax[2],
               buffer.alphaMax[3],
               buffer.alphaMax[4],
               buffer.alphaMax[5],
               buffer.alphaMax[6]
        );
        printf("    Alpha Min   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.alphaMin[0],
               buffer.alphaMin[1],
               buffer.alphaMin[2],
               buffer.alphaMin[3],
               buffer.alphaMin[4],
               buffer.alphaMin[5],
               buffer.alphaMin[6]
        );
        printf("    Vm          %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.vm[0],
               buffer.vm[1],
               buffer.vm[2],
               buffer.vm[3],
               buffer.vm[4],
               buffer.vm[5],
               buffer.vm[6]
        );
        printf("    On/Off      %5d\n", buffer.alphaOn);
    }
    //
    // FILTER_LARGE_CHANGE
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_LARGE_CHANGE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_LARGE_CHANGE: %d\n", buffer);
    }
    //
    // QUALITY
    //
    {
        QUALITY_PARAMETERS buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, QUALITY, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("QUALITY: %d, %d, %d, %d\n",
               buffer.error_offset,
               buffer.error_sensitivity,
               buffer.error_slope,
               buffer.filter_alpha
        );
    }


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // SetSensorParameter()
    //
    // This procedure allows the user to change selected sensor parameters. The
    // procedure is directed at a specific sensor by selecting that sensor with
    // the Sensor ID parameter. For the purpose of this exercise the ID = 0. This
    // is the 1st sensor in a system. This sensor will need to be attached for
    // these calls to respond without an error.
    //
    printf("\nSetSensorParameter()");
    printf("\n====================\n");
    printf("Using sensor ID = 0\n");
    sensorID = 0;

    // If we use the macros provided in SAMPLE2.H we can simplify
    // the sensor parameter setting as follows:

    printf("DATA_FORMAT:             DOUBLE_POSITION_ANGLES_TIME_STAMP\n");
    SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_ANGLES_TIME_STAMP, __LINE__);
    printf("ANGLE_ALIGN:             30, 45, 60\n"); {
        // initialize a structure of angles
        DOUBLE_ANGLES_RECORD anglesRecord = {30, 45, 60};
        SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord, __LINE__);
    }
    printf("HEMISPHERE:              TOP\n");
    SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, TOP, __LINE__);
    printf("FILTER_AC_WIDE_NOTCH:    true\n");
    SET_SENSOR_PARAMETER(sensorID, FILTER_AC_WIDE_NOTCH, true, __LINE__);
    printf("FILTER_AC_NARROW_NOTCH:  false\n");
    SET_SENSOR_PARAMETER(sensorID, FILTER_AC_NARROW_NOTCH, false, __LINE__);
    printf("FILTER_DC_ADAPTIVE:      1.0\n");
    SET_SENSOR_PARAMETER(sensorID, FILTER_DC_ADAPTIVE, 1.0, __LINE__);
    printf("FILTER_ALPHA_PARAMETERS:\n");
    printf("    alpha max    20000, 20000, 20000, 20000, 20000, 20000, 20000,\n");
    printf("    alpha min      500,   500,   500,   500,   500,   500,   500,\n");
    printf("    vm               2,     4,     8,    16,    32,    32,    32,\n");
    printf("    on/off        true\n"); {
        // initialize the alpha parameters
        ADAPTIVE_PARAMETERS adaptiveRecord = {
            500, 500, 500, 500, 500, 500, 500,
            20000, 20000, 20000, 20000, 20000, 20000, 20000,
            2, 4, 8, 16, 32, 32, 32,
            true
        };
        SET_SENSOR_PARAMETER(sensorID, FILTER_ALPHA_PARAMETERS, adaptiveRecord, __LINE__);
    }
    printf("FILTER_LARGE_CHANGE:     false\n");
    SET_SENSOR_PARAMETER(sensorID, FILTER_LARGE_CHANGE, false, __LINE__);
    printf("QUALITY:                 15, 20, 16, 5\n"); {
        // initialize the quality parameter structure
        QUALITY_PARAMETERS qualityParameters = {15, 20, 16, 5};
        SET_SENSOR_PARAMETER(sensorID, QUALITY, qualityParameters, __LINE__);
    }


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GetSensorParameter() [Repeat]
    //
    //
    printf("\nRepeat - GetSensorParameter()");
    printf("\n=============================\n");
    printf("Using sensor ID = 0\n");
    sensorID = 0;
    //
    // DATA_FORMAT
    //
    {
        DATA_FORMAT_TYPE buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, DATA_FORMAT, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("DATA_FORMAT: %d\n", buffer);
    }
    //
    // ANGLE_ALIGN
    //
    {
        DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, ANGLE_ALIGN, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("ANGLE_ALIGN: %6.2f, %6.2f, %6.2f\n",
               buffer.a,
               buffer.e,
               buffer.r);
    }
    //
    // HEMISPHERE
    //
    {
        HEMISPHERE_TYPE buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, HEMISPHERE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("HEMISPHERE: %4x\n", buffer);
    }
    //
    // FILTER_AC_WIDE_NOTCH
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_AC_WIDE_NOTCH, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_AC_WIDE_NOTCH: %d\n", buffer);
    }
    //
    // FILTER_AC_NARROW_NOTCH
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_AC_NARROW_NOTCH, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_AC_NARROW_NOTCH: %d\n", buffer);
    }
    //
    // FILTER_DC_ADAPTIVE
    //
    {
        double buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_DC_ADAPTIVE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_DC_ADAPTIVE: %5.2f\n", buffer);
    }
    //
    // FILTER_ALPHA_PARAMETERS
    //
    {
        ADAPTIVE_PARAMETERS buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_ALPHA_PARAMETERS, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_ALPHA_PARAMETERS:\n");
        printf("    Alpha max   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.alphaMax[0],
               buffer.alphaMax[1],
               buffer.alphaMax[2],
               buffer.alphaMax[3],
               buffer.alphaMax[4],
               buffer.alphaMax[5],
               buffer.alphaMax[6]
        );
        printf("    Alpha Min   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.alphaMin[0],
               buffer.alphaMin[1],
               buffer.alphaMin[2],
               buffer.alphaMin[3],
               buffer.alphaMin[4],
               buffer.alphaMin[5],
               buffer.alphaMin[6]
        );
        printf("    Vm          %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               buffer.vm[0],
               buffer.vm[1],
               buffer.vm[2],
               buffer.vm[3],
               buffer.vm[4],
               buffer.vm[5],
               buffer.vm[6]
        );
        printf("    On/Off      %5d\n", buffer.alphaOn);
    }
    //
    // FILTER_LARGE_CHANGE
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, FILTER_LARGE_CHANGE, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("FILTER_LARGE_CHANGE: %d\n", buffer);
    }
    //
    // QUALITY
    //
    {
        QUALITY_PARAMETERS buffer, *pBuffer = &buffer;
        errorCode = GetSensorParameter(sensorID, QUALITY, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("QUALITY: %d, %d, %d, %d\n",
               buffer.error_offset,
               buffer.error_sensitivity,
               buffer.error_slope,
               buffer.filter_alpha
        );
    }


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GetTransmitterParameter()
    //
    // This procedure allows the user to inspect selected transmitter parameters. The
    // procedure is directed at a specific transmitter by selecting that transmitter with
    // the Transmitter ID parameter. For the purpose of this exercise the ID = 0. This
    // is the 1st transmitter in a system.
    //
    printf("\nGetTransmitterParameter()");
    printf("\n=========================\n");
    printf("Using transmitter ID = 0\n");
    transmitterID = 0;
    //
    // REFERENCE_FRAME
    //
    {
        DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
        errorCode = GetTransmitterParameter(transmitterID, REFERENCE_FRAME, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("REFERENCE_FRAME: %6.2f, %6.2f, %6.2f\n",
               buffer.a,
               buffer.e,
               buffer.r);
    }

    //
    // XYZ_REFERENCE_FRAME
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetTransmitterParameter(transmitterID, XYZ_REFERENCE_FRAME, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("XYZ_REFERENCE_FRAME: %d\n", buffer);
    }


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // SetTransmitterParameter()
    //
    // This procedure allows the user to change selected transmitter parameters. The
    // procedure is directed at a specific transmitter by selecting that transmitter with
    // the Transmitter ID parameter. For the purpose of this exercise the ID = 0. This
    // is the 1st transmitter in a system.
    //
    printf("\nSetTransmitterParameter()");
    printf("\n=========================\n");
    printf("Using transmitter ID = 0\n");
    transmitterID = 0;

    // If we use the macros provided in SAMPLE2.H we can simplify
    // the transmitter parameter setting as follows:

    printf("REFERENCE_FRAME:         60, 45, 30\n"); {
        // initialize a structure of angles
        DOUBLE_ANGLES_RECORD anglesRecord = {60, 45, 30};
        SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord, __LINE__);
    }
    printf("XYZ_REFERENCE_FRAME:     true\n");
    SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, true, __LINE__);


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // GetTransmitterParameter() [Repeat]
    //
    //
    printf("\nRepeat - GetTransmitterParameter()");
    printf("\n==================================\n");
    printf("Using transmitter ID = 0\n");
    transmitterID = 0;
    //
    // REFERENCE_FRAME
    //
    {
        DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
        errorCode = GetTransmitterParameter(transmitterID, REFERENCE_FRAME, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("REFERENCE_FRAME: %6.2f, %6.2f, %6.2f\n",
               buffer.a,
               buffer.e,
               buffer.r);
    }
    //
    // XYZ_REFERENCE_FRAME
    //
    {
        BOOL buffer, *pBuffer = &buffer;
        errorCode = GetTransmitterParameter(transmitterID, XYZ_REFERENCE_FRAME, pBuffer, sizeof(buffer));
        if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
        printf("XYZ_REFERENCE_FRAME: %d\n", buffer);
    }

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // Search for the first attached transmitter and turn it on
    //
    for (id = 0; id < ATC3DG.m_config.numberTransmitters; id++) {
        if ((pXmtr + id)->m_config.attached) {
            // Transmitter selection is a system function.
            // Using the SELECT_TRANSMITTER parameter we send the id of the
            // transmitter that we want to run with the SetSystemParameter() call
            errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
            if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //  Restore angle align and reference frame default parameters so that
    //  the reported position is meaningful
    //
    printf("\nRestore defaults for sensor/transmitter #0");
    printf("\n==========================================\n");
    sensorID = 0;
    transmitterID = 0;
    // initialize a structure of angles
    DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 0};

    SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_ANGLES, __LINE__);
    SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord, __LINE__);
    SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, FRONT, __LINE__);

    SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord, __LINE__);
    SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, false, __LINE__);


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //  Save system setup to an initialization file
    //
}

DOUBLE_POSITION_ANGLES_RECORD readATI(short sensID) {
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // Collect data from all birds
    // Loop through all sensors and get a data record if the sensor is attached.
    // Print result to screen
    // Note: The default data format is DOUBLE_POSITION_ANGLES. We can use this
    // format without first setting it.
    //
    //
    DOUBLE_POSITION_ANGLES_RECORD record, *pRecord = &record;

    int errorCode = 0;

    // scan the sensors and request a record if the sensor is physically attached
    for (short sensorID = 0; sensorID < ATC3DG.m_config.numberSensors; sensorID++) {
        record.x = -100;
        record.y = -100;
        record.z = -100;
        record.a = -100;
        record.e = -100;
        record.r = -100;
        // sensor attached so get record
        errorCode = GetAsynchronousRecord(sensorID, pRecord, sizeof(record));
        if (errorCode != BIRD_ERROR_SUCCESS) {
            errorHandler(errorCode, __LINE__);
        }

        // get the status of the last data record
        // only report the data if everything is okay
        unsigned int status = GetSensorStatus(sensorID);

        if (status == VALID_STATUS) {
            // send output to console
            break;
        }
    }

    return record;
}
