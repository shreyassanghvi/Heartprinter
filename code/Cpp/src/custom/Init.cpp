//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/Init.h"
#include "../../include/custom/SystemController.h"

#include <iomanip>
//#include <unistd.h>

#include "../../include/Trackstar/ATC3DG.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>

#include <chrono>
#include <sstream>
#include <iostream>

#include <windows.h>
#include <stdio.h>

namespace fs = std::filesystem;

// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
// #define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// #define BAUDRATE                        57600

//user defined #define
#define RECORD_CNT                          10000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control

std::shared_ptr<spdlog::logger> create_dated_logger(bool make_default) {
    std::string log_dir = "../../logs/cpp";
    std::filesystem::create_directories(log_dir);

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

//Init commit for Control loop code
int main(int argc, char *argv[]) {
    // Set up logging
    auto logger = create_dated_logger(true);
    spdlog::info("Heart Printer System starting...");

    try {
        // Create system configuration
        SystemConfig config;
        config.deviceName = "/dev/ttyUSB0";  // Adjust for your system
        config.baudRate = 57600;
        config.recordCount = 1000;
        config.enableSafetyChecks = true;

        // Create and initialize system controller
        SystemController systemController(config);
        
        spdlog::info("Initializing system...");
        if (!systemController.initialize()) {
            spdlog::error("Failed to initialize system: {}", systemController.getLastError());
            return EXIT_FAILURE;
        }

        spdlog::info("System initialized successfully, starting main control loop...");
        
        // Run the main control loop
        systemController.run();
        
        // System will shut down automatically via RAII destructor
        spdlog::info("System execution completed");
        return EXIT_SUCCESS;
        
    } catch (const SystemException& e) {
        spdlog::error("System exception: [{}] {}", 
                     static_cast<int>(e.getErrorCode()), e.what());
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        spdlog::error("Unexpected exception: {}", e.what());
        return EXIT_FAILURE;
    } catch (...) {
        spdlog::error("Unknown exception occurred");
        return EXIT_FAILURE;
    }
}
