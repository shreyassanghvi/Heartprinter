#include "../../include/Trackstar/ATC3DG.h"
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
        // printf("%s %d\n", buffer, lineNum);
        spdlog::error("TrackStar: {} {}\n", buffer, lineNum);
        // printLog(LOG_ERROR, buffer);
        currentError = nextError;
    } while (currentError != BIRD_ERROR_SUCCESS);

    exit(0);
}

void initTxRx() {
    int errorCode; // used to hold error code returned from procedure call
    int i;
    int sensorID;
    int transmitterID;

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
    spdlog::info("Initializing ATC3DG system...");
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
    // printf("Number Boards:\t\t\t\t%d\n", ATC3DG.m_config.numberBoards);
    // printf("Number Sensors:\t\t\t\t%d\n", ATC3DG.m_config.numberSensors);
    // printf("Number Transmitters:\t\t\t%d\n", ATC3DG.m_config.numberTransmitters);
    spdlog::info("Boards: {}, Sensors: {}, Transmitters: {}", ATC3DG.m_config.numberBoards,
                 ATC3DG.m_config.numberSensors, ATC3DG.m_config.numberTransmitters);

    // printf("System AGC mode         = %d\n", ATC3DG.m_config.agcMode);
    // printf("Transmitter ID Running  = %d\n", ATC3DG.m_config.transmitterIDRunning);
    spdlog::info("AGC: {}, Transmitter: {}", static_cast<int>(ATC3DG.m_config.agcMode), ATC3DG.m_config.transmitterIDRunning);

    // printf("Maximum Range           = %6.2f\n", ATC3DG.m_config.maximumRange);
    // printf("Measurement Rate        = %10.6f\n", ATC3DG.m_config.measurementRate);
    // printf("Metric Mode             = %d\n", ATC3DG.m_config.metric);
    // printf("Line Frequency          = %6.2f\n", ATC3DG.m_config.powerLineFrequency);
    spdlog::info("Range: {}, Rate: {}, Metric: {}, Line: {}",
                 ATC3DG.m_config.maximumRange, ATC3DG.m_config.measurementRate,
                 ATC3DG.m_config.metric, ATC3DG.m_config.powerLineFrequency);

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
    // printf("\nCurrent System Parameters:");
    // printf("\n==========================================\n");
    // //
    // // SELECT_TRANSMITTER
    // //
    // auto SYS_TX_ID = GET_SYSTEM_PARAMETER<short>(SELECT_TRANSMITTER);
    // // printf("SELECT_TRANSMITTER:\t\t\t%d\n", SYS_TX_ID);
    // spdlog::info("SELECT_TRANSMITTER: {}", SYS_TX_ID);
    // //
    // // POWER_LINE_FREQUENCY
    // //
    // auto SYS_POWER_LINE_FREQUENCY = GET_SYSTEM_PARAMETER<double>(POWER_LINE_FREQUENCY);
    // // printf("POWER_LINE_FREQUENCY:\t\t\t%5.2f\n", SYS_POWER_LINE_FREQUENCY);
    // spdlog::info("POWER_LINE_FREQUENCY: {}", SYS_POWER_LINE_FREQUENCY);
    // //
    // // AGC_MODE
    // //
    // auto SYS_AGC_MODE = GET_SYSTEM_PARAMETER<int>(AGC_MODE);
    // // printf("AGC_MODE:\t\t\t\t%s\n", SYS_AGC_MODE == 1 ? "SENSOR_AGC_ONLY" : "TRANSMITTER_AND_SENSOR_AGC");
    // spdlog::info("AGC_MODE: {}", SYS_AGC_MODE == 1 ? "SENSOR_AGC_ONLY" : "TRANSMITTER_AND_SENSOR_AGC");
    // //
    // // MEASUREMENT_RATE
    // //
    // auto SYS_MEAS_RATE = GET_SYSTEM_PARAMETER<double>(MEASUREMENT_RATE);
    // // printf("MEASUREMENT_RATE:\t\t\t%5.2f\n", SYS_MEAS_RATE);
    //
    // //
    // // MAXIMUM_RANGE
    // //
    // auto SYS_MAX_RANGE = GET_SYSTEM_PARAMETER<double>(MAXIMUM_RANGE);
    // printf("MAXIMUM_RANGE:\t\t\t\t%5.2f\n", SYS_MAX_RANGE);
    // //
    // // METRIC
    // //
    // auto SYS_METRIC = GET_SYSTEM_PARAMETER<BOOL>(METRIC);
    // printf("METRIC:\t\t\t\t\t%s\n", SYS_METRIC == 1 ? "TRUE" : "FALSE");



    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //	The previous calls can simplified using the macros provided in SAMPLE2.H
    //  (printf statements are provided for clarity)
    //
    // printf("Set System Parameters:");
    spdlog::info("Set System Parameters:");
    // printf("\n==========================================\n");
    // printf("SELECT_TRANSMITTER:\t\t\t0\n");
    SET_SYSTEM_PARAMETER(SELECT_TRANSMITTER, 0, __LINE__);
    spdlog::info("SELECT_TRANSMITTER: {}", 0);
    // printf("POWER_LINE_FREQUENCY:\t\t\t50.0\n");
    SET_SYSTEM_PARAMETER(POWER_LINE_FREQUENCY, 50.0, __LINE__);
    spdlog::info("POWER_LINE_FREQUENCY: {}", 50.0);
    // printf("AGC_MODE:\t\t\t\tSENSOR_AGC_ONLY\n");
    SET_SYSTEM_PARAMETER(AGC_MODE, SENSOR_AGC_ONLY, __LINE__);
    spdlog::info("AGC_MODE: SENSOR_AGC_ONLY");
    // printf("MEASUREMENT_RATE:\t\t\t75.0\n");
    SET_SYSTEM_PARAMETER(MEASUREMENT_RATE, 75.0, __LINE__);
    spdlog::info("MEASUREMENT RATE SET: {}",75.0);
    // printf("MAXIMUM_RANGE:\t\t\t\t72.0\n");
    SET_SYSTEM_PARAMETER(MAXIMUM_RANGE, 72.0, __LINE__);
    spdlog::info("MAXIMUM RANGE SET: {}",72.0);
    // printf("METRIC:\t\t\t\t\tTRUE\n");
    SET_SYSTEM_PARAMETER(METRIC, true, __LINE__);
    spdlog::info("METRIC SET: {}",true);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    //  Restore angle align and reference frame default parameters so that
    //  the reported position is meaningful
    //
    // printf("\nSetting Defaults for Sensors");
    spdlog::info("Setting Defaults for Sensors");
    // printf("\n==========================================\n");
    // initialize a structure of angles
    DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 0};
    for (sensorID = 0; sensorID < ATC3DG.m_config.numberSensors; sensorID++) {
        // printf("Sensor %d\n", sensorID);
        spdlog::info("Sensor {}", sensorID);
        // printf("Data Format:\t\t\t\t12 (DOUBLE_POSITION_ANGLES)\n");
        SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_ANGLES, __LINE__);
        spdlog::info("Data Format: {}", static_cast<int>(DOUBLE_POSITION_ANGLES));
        // printf("Angle Align:\t\t\t\ta: %lf, e: %lf, r: %lf\n", );
        SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord, __LINE__);
        spdlog::info("Angle Align: a({}), e({}), r({})", anglesRecord.a, anglesRecord.e, anglesRecord.r);
        // printf("HEMISPHERE:\t\t\t\t0 (FRONT)");
        SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, FRONT, __LINE__);
        spdlog::info("HEMISPHERE: {}", static_cast<int>(FRONT));
        // printf("\n=============================\n");
    }


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
    printf("\nGet Sensor Parameters");
    printf("\n==========================================\n");
    for (sensorID = 0; sensorID < ATC3DG.m_config.numberSensors; sensorID++) {
        printf("Sensor ID: %d\n", sensorID);
        //
        // DATA_FORMAT
        //
        auto SENSOR_DATA_FORMAT = GET_SENSOR_PARAMETER<DATA_FORMAT_TYPE>(sensorID, DATA_FORMAT);
        printf("DATA_FORMAT: %d\n", SENSOR_DATA_FORMAT);
        //
        // ANGLE_ALIGN
        //
        auto SENSOR_ANGLE_ALIGN = GET_SENSOR_PARAMETER<DOUBLE_ANGLES_RECORD>(sensorID, ANGLE_ALIGN);
        printf("ANGLE_ALIGN: %6.2f, %6.2f, %6.2f\n", SENSOR_ANGLE_ALIGN.a, SENSOR_ANGLE_ALIGN.e, SENSOR_ANGLE_ALIGN.r);
        //
        // HEMISPHERE
        //
        auto SENSOR_HEMISPHERE = GET_SENSOR_PARAMETER<HEMISPHERE_TYPE>(sensorID, HEMISPHERE);
        printf("HEMISPHERE: %4x\n", SENSOR_HEMISPHERE);
        //
        // FILTER_AC_WIDE_NOTCH
        //
        auto SENSOR_FILTER_AC_WIDE_NOTCH = GET_SENSOR_PARAMETER<BOOL>(sensorID, FILTER_AC_WIDE_NOTCH);
        printf("FILTER_AC_WIDE_NOTCH: %d\n", SENSOR_FILTER_AC_WIDE_NOTCH);
        //
        // FILTER_AC_NARROW_NOTCH
        //
        auto SENSOR_FILTER_AC_NARROW_NOTCH = GET_SENSOR_PARAMETER<BOOL>(sensorID, FILTER_AC_NARROW_NOTCH);
        printf("FILTER_AC_NARROW_NOTCH: %d\n", SENSOR_FILTER_AC_NARROW_NOTCH);
        //
        // FILTER_DC_ADAPTIVE
        //
        auto SENSOR_FILTER_DC_ADAPTIVE = GET_SENSOR_PARAMETER<double>(sensorID, FILTER_DC_ADAPTIVE);
        printf("FILTER_DC_ADAPTIVE: %5.2f\n", SENSOR_FILTER_DC_ADAPTIVE);
        //
        // FILTER_ALPHA_PARAMETERS
        //
        auto SENSOR_FILTER_ALPHA_PARAMETERS = GET_SENSOR_PARAMETER<ADAPTIVE_PARAMETERS>(
            sensorID, FILTER_ALPHA_PARAMETERS);
        printf("FILTER_ALPHA_PARAMETERS:\n");
        printf("    Alpha max   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[0],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[1],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[2],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[3],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[4],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[5],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMax[6]
        );
        printf("    Alpha Min   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[0],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[1],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[2],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[3],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[4],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[5],
               SENSOR_FILTER_ALPHA_PARAMETERS.alphaMin[6]
        );
        printf("    Vm          %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[0],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[1],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[2],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[3],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[4],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[5],
               SENSOR_FILTER_ALPHA_PARAMETERS.vm[6]
        );
        printf("    On/Off      %5d\n", SENSOR_FILTER_ALPHA_PARAMETERS.alphaOn);
        //
        // FILTER_LARGE_CHANGE
        //
        auto SENSOR_FILTER_LARGE_CHANGE = GET_SENSOR_PARAMETER<BOOL>(sensorID, FILTER_LARGE_CHANGE);
        printf("FILTER_LARGE_CHANGE: %d\n", SENSOR_FILTER_LARGE_CHANGE);

        //
        // QUALITY
        //
        auto SENSOR_QUALITY = GET_SENSOR_PARAMETER<QUALITY_PARAMETERS>(sensorID, QUALITY);
        printf("QUALITY: %d, %d, %d, %d",
               SENSOR_QUALITY.error_offset,
               SENSOR_QUALITY.error_sensitivity,
               SENSOR_QUALITY.error_slope,
               SENSOR_QUALITY.filter_alpha
        );
        printf("\n=============================\n");
    }

    printf("\nSetting Defaults for Transmitters");
    printf("\n==========================================\n");
    transmitterID = 0;
    printf("Transmitter %d\n", transmitterID);
    printf("REFERENCE_FRAME:\t\t\ta: %lf, e: %lf, r: %lf\n", anglesRecord.a, anglesRecord.e, anglesRecord.r);
    SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord, __LINE__);
    printf("XYZ_REFERENCE_FRAME:\t\t\tFALSE");
    SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, false, __LINE__);
    printf("\n==========================================\n");

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
    printf("\nGet Transmitter Parameter");
    printf("\n==========================================\n");
    printf("Using transmitter ID = 0\n");
    transmitterID = 0;
    //
    // REFERENCE_FRAME
    //
    auto referenceFrame = GET_TRANSMITTER_PARAMETER<DOUBLE_ANGLES_RECORD>(transmitterID, REFERENCE_FRAME);
    printf("REFERENCE_FRAME: %6.2f, %6.2f, %6.2f\n",
           referenceFrame.a,
           referenceFrame.e,
           referenceFrame.r);
    //
    // XYZ_REFERENCE_FRAME
    //
    auto xyzReferenceFrame = GET_TRANSMITTER_PARAMETER<BOOL>(transmitterID, XYZ_REFERENCE_FRAME);
    printf("XYZ_REFERENCE_FRAME: %d\n", xyzReferenceFrame);
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
            //TODO: LOGGING
            break;
        }
    }

    return record;
}

int getConnectedSensors() {
    return ATC3DG.m_config.numberSensors;
}
