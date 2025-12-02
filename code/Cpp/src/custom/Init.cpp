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
#include <signal.h>
#include <atomic>

#include <windows.h>
#include <stdio.h>

namespace fs = std::filesystem;

//user defined #define
#define RECORD_CNT                          10000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control

// Global pointer to SystemController for signal handler
static SystemController* g_systemControllerForSignal = nullptr;
static std::atomic<bool> g_shutdownInProgress(false);

#ifdef _WIN32
// Windows console control handler for Ctrl+C, Ctrl+Break, and close events
BOOL WINAPI consoleControlHandler(DWORD ctrlType) {
    // DEBUG: Write to stderr immediately to see if handler is called
    // Prevent re-entrant calls during shutdown
    bool expected = false;
    if (!g_shutdownInProgress.compare_exchange_strong(expected, true)) {
        // Shutdown already in progress
        fprintf(stderr, "*** Shutdown already in progress ***\n");
        fflush(stderr);
        return TRUE;
    }

    const char* eventName = "UNKNOWN";
    switch (ctrlType) {
        case CTRL_C_EVENT:
            eventName = "Ctrl+C";
            break;
        case CTRL_BREAK_EVENT:
            eventName = "Ctrl+Break";
            break;
        case CTRL_CLOSE_EVENT:
            eventName = "Console Close";
            break;
        case CTRL_LOGOFF_EVENT:
            eventName = "User Logoff";
            break;
        case CTRL_SHUTDOWN_EVENT:
            eventName = "System Shutdown";
            break;
        default:
            break;
    }

    spdlog::warn("{} received - initiating graceful shutdown...", eventName);

    if (g_systemControllerForSignal) {
        // If system controller exists, do graceful shutdown
        g_systemControllerForSignal->shutdown();
        spdlog::info("Cleanup completed, exiting...");
        Sleep(1000);
    } else {
        // Called before system controller exists
        spdlog::info("Shutdown requested during initialization, exiting...");
        Sleep(100);
    }

    // Must exit the program, returning TRUE just tells Windows we handled it
    // but doesn't stop the program
    exit(0);
}
#endif

// Signal handler for various termination signals (fallback for non-console signals)
void signalHandler(int signal) {
    // Prevent re-entrant calls during shutdown
    bool expected = false;
    if (!g_shutdownInProgress.compare_exchange_strong(expected, true)) {
        // Shutdown already in progress, just return
        return;
    }

    const char* signalName = "UNKNOWN";
    switch (signal) {
        case SIGINT:
            signalName = "SIGINT (Ctrl+C)";
            break;
        case SIGTERM:
            signalName = "SIGTERM (Termination request)";
            break;
        case SIGBREAK:
            signalName = "SIGBREAK (Ctrl+Break)";
            break;
        case SIGABRT:
            signalName = "SIGABRT (Abort)";
            break;
        default:
            break;
    }

    spdlog::warn("{} received - initiating graceful shutdown...", signalName);

    if (g_systemControllerForSignal) {
        g_systemControllerForSignal->shutdown();
    }

    spdlog::info("Cleanup completed, exiting...");
    exit(0);
}

bool getUserConfirmation(const std::string& prompt) {
    std::string input;
    while (true) {
        std::cout << prompt << " (y/n): ";
        std::getline(std::cin, input);

        if (!input.empty()) {
            char response = std::tolower(input[0]);
            if (response == 'y') {
                return true;
            } else if (response == 'n') {
                return false;
            }
        }
        std::cout << "Invalid input. Please enter 'y' or 'n'." << std::endl;
    }
}

// Pre-initialization checklist
// Returns true if all checks pass, false if user responds 'n' to any check
bool runPreInitializationChecklist() {
    std::cout << "Heart Printer Pre-Initialization Checklist" << std::endl;

    if (!getUserConfirmation("1. Is the power to the system on?")) {
        spdlog::error("Pre-check failed: System power not confirmed. Please restart from the beginning.");
        return false;
    }
    spdlog::info("Check passed: System power confirmed");

    if (!getUserConfirmation("2. Is the ESTOP off?")) {
        spdlog::error("Pre-check failed: ESTOP status not confirmed. Please restart from the beginning.");
        return false;
    }
    spdlog::info("Check passed: ESTOP confirmed off");

    if (!getUserConfirmation("3. Is the TrackStar beacon next to the patient?")) {
        spdlog::error("Pre-check failed: TrackStar beacon placement not confirmed. Please restart from the beginning.");
        return false;
    }
    spdlog::info("Check passed: TrackStar beacon placement confirmed");

    if (!getUserConfirmation("4. Are the Loadcell power switches set to on?")) {
        spdlog::error("Pre-check failed: Loadcell power not confirmed. Please restart from the beginning.");
        return false;
    }
    spdlog::info("Check passed: Loadcell power confirmed");

    if (!getUserConfirmation("5. Did you power cycle the motors?")) {
        spdlog::error("Pre-check failed: Motor power cycle not confirmed. Please restart from the beginning.");
        return false;
    }
    spdlog::info("Check passed: Motor power cycle confirmed");

    std::cout << "All pre-checks passed! Proceeding..." << std::endl;

    return true;
}

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
// #ifdef _WIN32
//     // Ensure console is attached (required for proper stdout/stderr when run from terminal)
//     if (AttachConsole(ATTACH_PARENT_PROCESS) || AllocConsole()) {
//         FILE* pFile = nullptr;
//         freopen_s(&pFile, "CONOUT$", "w", stdout);
//         freopen_s(&pFile, "CONOUT$", "w", stderr);
//         freopen_s(&pFile, "CONIN$", "r", stdin);
//     }
// #endif
    // Set up logging
    auto logger = create_dated_logger(true);
    spdlog::info("Heart Printer System starting...");

    // Set up signal handlers for graceful shutdown
#ifdef _WIN32
    // Use Windows console control handler (more reliable than signal() on Windows)
    if (!SetConsoleCtrlHandler(consoleControlHandler, TRUE)) {
        DWORD error = GetLastError();
        spdlog::error("Failed to set console control handler! Error code: {}", error);
    } else {
        spdlog::info("Console control handler registered (Ctrl+C, Ctrl+Break, Close)");
    }
#endif

    // Also register standard signal handlers as fallback
    signal(SIGINT, signalHandler);   // Ctrl+C (fallback)
    signal(SIGTERM, signalHandler);  // Termination request
    signal(SIGABRT, signalHandler);  // Abort signal
    spdlog::info("Signal handlers registered (SIGINT, SIGTERM, SIGABRT)");

    try {
        spdlog::info("Setting up config");
        // Create system configuration
        SystemConfig config;
        config.deviceName = "COM4";  // Adjust for your system
        config.baudRate = 57600;
        config.enableSafetyChecks = true;
        config.maxLoadVoltage = -0.01;
        config.minLoadVoltage = -0.03;
        config.tensionAdjustmentSteps = 50;
        config.trackingDeadband = 300.0;
        config.posErrorThreshold = 1.0;
        config.angleThreshold = 7.0;

        // Run pre-initialization checklist
        if (!runPreInitializationChecklist()) {
            spdlog::error("Pre-initialization checklist failed. Exiting. Please restart from the beginning.");
            return EXIT_FAILURE;
        }

        // Create and initialize system controller
        spdlog::info("Construct systemcontroller");
        SystemController systemController(config);

        // Set global pointer for signal handler
        g_systemControllerForSignal = &systemController;

        spdlog::info("Initializing system...");
        if (!systemController.initialize()) {
            spdlog::error("Failed to initialize system: {}", systemController.getLastError());
            return EXIT_FAILURE;
        }

        spdlog::info("System initialized successfully, starting main control loop...");
        
        // Run the main control loop
        systemController.run();

        // Clear global pointer before destruction
        g_systemControllerForSignal = nullptr;

        // System will shut down automatically via RAII destructor
        spdlog::info("System execution completed");
        return EXIT_SUCCESS;
        
    } catch (const SystemException& e) {
        g_systemControllerForSignal = nullptr;
        spdlog::error("System exception: [{}] {}",
                     static_cast<int>(e.getErrorCode()), e.what());
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        g_systemControllerForSignal = nullptr;
        spdlog::error("Unexpected exception: {}", e.what());
        return EXIT_FAILURE;
    } catch (...) {
        g_systemControllerForSignal = nullptr;
        spdlog::error("Unknown exception occurred");
        return EXIT_FAILURE;
    }
}
