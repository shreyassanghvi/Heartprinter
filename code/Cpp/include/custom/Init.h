//
// Created by shreyas on 1/19/25.
//

#ifndef INIT_H
#define INIT_H
// #if defined(__linux__) || defined(__APPLE__)
// #include <fcntl.h>
// #include <termios.h>
// #define STDIN_FILENO 0
// #elif defined(_WIN32) || defined(_WIN64)
//
// #endif

#include <conio.h>
//ATI Trackstar headers
#include "../Trackstar/StdAfx.h"
#include "../Trackstar/ATC3DG.h"
#include "CTrackstar.h"

// NI DAQmx headers
#include "NIDAQ.h"

//Motor SDK Headers
#include "../Dynamixel_SDK/dynamixel_sdk.h"

// userdefined headers
#include "log.h"

#include "Motor.h"

enum STATES {
    START,
    INIT_DAQ,
    INIT_MOTORS,
    INIT_TRACKSTAR,
    INIT_LOADCELL,
    READ_TRACKSTAR,
    READ_DAQ,
    READ_MOTORS,
    SET_MOTOR,
    MOVE_MOTORS,
    ERR,
    CLEANUP_DAQ,
    CLEANUP_MOTORS,
    CLEANUP_TRACKSTAR,
    END
};

#endif //INIT_H
