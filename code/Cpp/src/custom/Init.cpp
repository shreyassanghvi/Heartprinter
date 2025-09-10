//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/Init.h"

#include <iomanip>
//#include <unistd.h>

#include "../../include/Trackstar/ATC3DG.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>

#include <chrono>
#include <sstream>
#include <iostream>

namespace fs = std::filesystem;
// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
// #define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// #define BAUDRATE                        57600

//user defined #define
#define RECORD_CNT                          1000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control

enum TRACKSTAR_SENSORS_ID {
    LEFT_SENSOR = 0,
    BASE_SENSOR,
    RIGHT_SENSOR,
    MOVING_BASE_SENSOR,
};

STATES state;
// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);


int setupMotorPort() {
    if (portHandler->openPort()) {
        spdlog::info("Succeeded to open the port!");
    } else {
        spdlog::error("Failed to open the port!");
        spdlog::error("Press any key to terminate...");
        _getch();
        return EXIT_FAILURE;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        spdlog::info("Succeeded to change the baudrate!");
    } else {
        spdlog::error("Failed to change the baudrate!");
        spdlog::error("Press any key to terminate...");
        _getch();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int initMotors(Motor motors) {
    uint8_t dxl_error = 0; // Dynamixel error
    motors.setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE);
    // Enable Dynamixel#i Torque
    dxl_error += motors.enableTorque(packetHandler, portHandler);
    if (dxl_error != 0) {
        spdlog::error("Failed to enable torque for motor #{}", motors.getMotorID());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void cleanUpAndExit() {
    int errorCode = SaveSystemConfiguration("newfile.ini");
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // Turn off the transmitter before exiting
    // We turn off the transmitter by "selecting" a transmitter with an id of "-1"
    //
    short id = -1;
    errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
}

void DataHandler(double *data, uInt32 numSamples) {
    try {
        if (!data || numSamples == 0) {
            spdlog::warn("Invalid data or sample count in DataHandler!");
            return;
        }
        // printf("Test\n");
        // printf("Received %u samples. First: %.3f\n", numSamples, data[0]);
        // if (data == nullptr) {
        //     printf("No data\n");
        //     return;
        // }
        std::vector vData(data, data + numSamples);
        spdlog::info("DAQ - Sample Count: {} - Received Data[0] - {:8.5f}",
                     numSamples,
                     data[0]);
    } catch (std::exception &e) {
        spdlog::error("Error: {}", e.what());

        state = ERR;
    }
}

void ErrorHandlerDAQ(const char *errorMessage) {
    spdlog::error("DAQ Error: {}", errorMessage);
    state = ERR;
}

std::shared_ptr<spdlog::logger> create_dated_logger(bool make_default) {
    std::string log_dir = "../../logs/cpp";
    std::filesystem::create_directories("log_dir");

    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << log_dir<<"/log_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << ".txt";

    std::string filename = oss.str();
    try {
        // Create sinks: file sink and color console sink
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};

        // Create logger with both sinks
        auto logger = std::make_shared<spdlog::logger>("multi_sink", sinks.begin(), sinks.end());

        if (make_default) {
            register_logger(logger);
            set_default_logger(logger);
        }
        spdlog::set_level(spdlog::level::debug);
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$]:\t%v"); // Coloring enabled with %^
        spdlog::info("Logger initialized with both file and console sinks");
        return logger;
    } catch (const std::exception &e) {
        spdlog::error("Failed to create logger: {}", e.what());
        return nullptr;
    }
}

class StateController {
public:
    std::vector<Motor> vMotors;
    int dxl_comm_result; // Communication result
    int min_pos = DXL_MINIMUM_POSITION_VALUE;
    int max_pos = static_cast<int>(DXL_MAXIMUM_POSITION_VALUE * 1.5);
    int neutral_pos = (min_pos + max_pos) / 2;
    uint8_t dxl_error; // Dynamixel error
    long data_count = 0;
    int ERR_FLG = 0;
    int motor_destination[MOTOR_CNT];
    STATES current_state;

    DAQSystem daqSystem;
    DOUBLE_POSITION_ANGLES_RECORD currentPosition{};


    StateController() {
        current_state = START;
    }

    // These are the ideal state transitions
    void processStateTransition() {
        switch (current_state) {
        case START:
            current_state = INIT;
            break;
        case INIT:
            current_state = READY;
            break;
        case READY:
            current_state = MOVE;
            break;
        case MOVE:
            current_state = READY;
            break;
        case ERR:
            current_state = END;
            break;
        case END:
            current_state = CLEANUP;
            break;
        }
    }

    void startup() {
        dxl_comm_result = COMM_SUCCESS;
        groupSyncWrite.clearParam();
        create_dated_logger(true);
    }

    bool initialize() {
        current_state = INIT;
        // BEGIN REGION DAQ INIT
        spdlog::info("Initializing the daq...");
        spdlog::info("NEW LOGGING TEST");
        DigitalConfig digiConfig = { "Dev1", "PFI0:2" };
        AnalogConfig analogConfig = {
            "Dev1", "ai0:2",
            0.0, 10.0,
            10000.0,
            1000
        };
        spdlog::info("DigitalConfig - Device: {}, Channel: {}", digiConfig.device, digiConfig.channel);

        spdlog::info("AnalogConfig - Device: {}, Channel: {}, MinVoltage: {:.2f}, MaxVoltage: {:.2f}, SampleRate: {:.2f}, SamplesPerCallback: {}",
            analogConfig.device, analogConfig.channel,
            analogConfig.minVoltage, analogConfig.maxVoltage,
            analogConfig.sampleRate, analogConfig.samplesPerCallback);

        daqSystem = initDAQSystem(digiConfig, analogConfig, DataHandler, ErrorHandlerDAQ);
        if (!daqSystem.initialized) {
            spdlog::error("Failed to initialize DAQ system");
            return false;
        }
        if (setAllLEDs(daqSystem.digitalTask, LED_ON) != EXIT_SUCCESS) {
            spdlog::error("Failed to set all LEDs off");
            return false;
        }
        if (setAllLEDs(daqSystem.digitalTask, LED_OFF) != EXIT_SUCCESS) {
            spdlog::error("Failed to set all LEDs off");
            return false;
        }

        // END REGION DAQ INIT

        // BEGIN REGION MOTORS INIT
        spdlog::info("Initializing motors...");
        if (setupMotorPort() != EXIT_SUCCESS) {
            return false;
        }
        for (int i = 0; i < MOTOR_CNT; i++) {
            motor_destination[i] = 0;
            vMotors.emplace_back(i);
            if (initMotors(vMotors.back()) != EXIT_SUCCESS) {
                return false;
            }
            if (vMotors.back().setMotorDestination(&groupSyncWrite, DXL_MINIMUM_POSITION_VALUE) != true) {
                return false;
            }
        }
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        groupSyncWrite.clearParam();
        // END REGION MOTORS INIT

        // BEGIN REGION TRACKSTAR INIT
        initTxRx();
        // END REGION TRACKSTAR INIT

        // BEGIN REGION LOADCELL INIT
        spdlog::warn("TODO: Initializing load cell...");
        DAQStart(daqSystem.analogHandle);
        // END REGION LOADCELL INIT

        current_state = READY;
        return true;
    }

    void handleError() {
        current_state = ERR;
        ERR_FLG = 1;
        spdlog::error("User error");
        // Do other ERR stuff if necessary
        current_state = END;
    }

    bool updateCurrentPositionFromTrackstar() {
        if (data_count == 0) {
            spdlog::info("Collect {} Data records from Sensors", RECORD_CNT);

            spdlog::info("Metric mode was selected, position is in mm.");
        }
        if (data_count < RECORD_CNT) {
            for (auto i = 0; i < getConnectedSensors(); i++) {
                currentPosition = readATI(i);
                data_count++;
                std::string sensorName;
                switch (i) {
                case LEFT_SENSOR:
                    sensorName = "L";
                    break;
                case RIGHT_SENSOR:
                    sensorName = "R";
                    break;
                case BASE_SENSOR:
                    sensorName = "C";
                    break;
                case MOVING_BASE_SENSOR:
                    sensorName = "M";
                    break;
                default:
                    sensorName = "Unknown";
                    spdlog::error("Incorrect sensor ID");
                    return false;
                    break;
                }

                spdlog::info("TS: [{}] {:8.3f} {:8.3f} {:8.3f} : {:8.3f} {:8.3f} {:8.3f}",
                    sensorName,
                    currentPosition.x, currentPosition.y, currentPosition.z,
                    currentPosition.a, currentPosition.e, currentPosition.r);
            }
        }
        else {
            current_state = END;
        }

        return true;
    };

    void setMotorPositions() {
        current_state = MOVE;
        static const double DEADBAND = 3.0;
        static const double REF_X = 0.0;
        static const double REF_Y = 0.0;
        static const double REF_Z = 0.0;

        double dx = currentPosition.x - REF_X;
        double dy = currentPosition.y - REF_Y;
        double dz = currentPosition.z - REF_Z;

        // X axis (motor 0): normal logic
        if (dx > DEADBAND) {
            motor_destination[0] = max_pos; // forward
            setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
        }
        else if (dx < -DEADBAND) {
            motor_destination[0] = min_pos; // backward
            setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
        }
        else {
            motor_destination[0] = neutral_pos; // neutral
            setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_OFF);
        }

        // Y axis (motor 1): INVERTED logic
        if (dy > DEADBAND) {
            motor_destination[1] = min_pos; // backward (inverted)
            setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
        }
        else if (dy < -DEADBAND) {
            motor_destination[1] = max_pos; // forward (inverted)
            setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
        }
        else {
            motor_destination[1] = neutral_pos; // neutral
            setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_OFF);
        }

        // Z axis (motor 2): normal logic
        if (dz > DEADBAND) {
            motor_destination[2] = max_pos; // forward
            setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
        }
        else if (dz < -DEADBAND) {
            motor_destination[2] = min_pos; // backward
            setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
        }
        else {
            motor_destination[2] = neutral_pos; // neutral
            setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_OFF);
        }

        spdlog::info("dx: {:4.2f}, dy: {:4.2f}, dz: {:4.2f} | Dest: [{:4d}, {:4d}, {:4d}]",
            dx, dy, dz,
            motor_destination[0], motor_destination[1], motor_destination[2]);
    }

    bool moveMotorPositions() {
        for (int j = 0; j < MOTOR_CNT; j++) {
            if (vMotors[j].setMotorDestination(&groupSyncWrite, motor_destination[j]) != true) {
                return false;
            }
        }

        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        groupSyncWrite.clearParam();

        return true;
    }

    bool readMotorPositions() {
        dxl_error = 0;
        dxl_comm_result = 0;
        bool notAtPosition = true;
        do {
            // groupSyncRead.clearParam();
            // Sync read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
                for (int i = 0; i < MOTOR_CNT; i++) {
                    if (groupSyncRead.getError(vMotors[i].getMotorID(), &dxl_error)) {
                        spdlog::error("[ID:{:3d}] {}\n", vMotors[i].getMotorID(),
                            packetHandler->getRxPacketError(dxl_error));
                    }
                }
                return false;
            }

            // Reset exitFlag to true at the start of each iteration
            notAtPosition = true;

            for (int i = 0; i < MOTOR_CNT; i++) {
                // Check if each motor is at its destination
                // If any motor is not at destination, set exitFlag to false
                // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t", vMotors[i].getMotorID(),
                //        motor_destination[i], vMotors[i].checkAndGetPresentPosition(&groupSyncRead));
                vMotors[i].checkAndGetPresentPosition(&groupSyncRead);
                if (!vMotors[i].checkIfAtGoalPosition(motor_destination[i])) {
                    notAtPosition = false;
                }
            }

            // Optional: Add a small delay to prevent tight looping
            // usleep(10000); // 10ms delay, adjust as needed
        } while (notAtPosition); // Continue loop until all motors reach destination

        current_state = READY;
        return true;
    }

    bool cleanUp() {
        current_state = CLEANUP;
        setAllLEDs(daqSystem.digitalTask, LED_OFF);
        DAQStop(daqSystem.analogHandle);
        cleanupDAQSystem(daqSystem);

        groupSyncWrite.clearParam();
        for (auto motor : vMotors) {
            motor.disableTorque(packetHandler, portHandler);
        }
        // vMotors.clear();
        portHandler->closePort();
        cleanUpAndExit();

        if (ERR_FLG != 0) {
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }
};

//Init commit for Control loop code
int main(int argc, char *argv[]) {
    StateController state_controller;

    // Start up our state controller
    // We know that the state machine should be:
    // Start -> Init
    // Init -> Ready; Init -> Err
    // Ready -> Move; Ready -> Err
    // Move -> Ready; Move -> Err; Move -> End
    // Err -> End
    // End -> Cleanup

    // Start state controller
    state_controller.startup();

    // Initialize the state controller
    if (!state_controller.initialize()) {
        state_controller.handleError();
        return state_controller.cleanUp();
    }

    // TODO: We can probably move this loop into the state controller
    while (true) {
        // Get latest position from trackstar
        if (state_controller.current_state != READY || !state_controller.updateCurrentPositionFromTrackstar()) {
            // Handle ERR appropriately
            state_controller.handleError();
        }

        // End it if there was an error or we're just good to end
        if (state_controller.current_state == END) {
            return state_controller.cleanUp();
        }

        // Calculate the motor positions from DEADBAND
        state_controller.setMotorPositions();

        // Try to move motors
        if (!state_controller.moveMotorPositions()) {
            state_controller.handleError();
            return state_controller.cleanUp();
        }

        // Wait until we get to goal position
        if (!state_controller.readMotorPositions()) {
            state_controller.handleError();
            return state_controller.cleanUp();
        }
    }
}
