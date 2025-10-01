#include <windows.h>
#include <iostream>
#include <cstring>
#include <string>

HANDLE g_hMapFile = NULL;
void* g_pBuf = NULL;

BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_CLOSE_EVENT || signal == CTRL_BREAK_EVENT) {
        if (g_pBuf) {
            UnmapViewOfFile(g_pBuf);
            g_pBuf = NULL;
        }
        if (g_hMapFile) {
            CloseHandle(g_hMapFile);
            g_hMapFile = NULL;
        }
        std::cout << "\nCleaned up shared memory handles on signal.\n";
        ExitProcess(0);
    }
    return FALSE;
}

int main_bk() {
    const char* shmName = "Local\\CPPToPy";
    const size_t shmSize = 1024;

    // Register console control handler
    if (!SetConsoleCtrlHandler(ConsoleHandler, TRUE)) {
        std::cerr << "Could not set control handler" << std::endl;
        return 1;
    }

    g_hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,
        NULL,
        PAGE_READWRITE,
        0,
        (DWORD)shmSize,
        shmName);

    if (g_hMapFile == NULL) {
        std::cerr << "CreateFileMapping failed: " << GetLastError() << std::endl;
        return 1;
    }

    g_pBuf = MapViewOfFile(
        g_hMapFile,
        FILE_MAP_ALL_ACCESS,
        0,
        0,
        shmSize);

    if (g_pBuf == NULL) {
        std::cerr << "MapViewOfFile failed: " << GetLastError() << std::endl;
        CloseHandle(g_hMapFile);
        return 1;
    }

    std::cout << "Shared memory created. Enter text to write, Ctrl+C to exit." << std::endl;

    try {
        while (true) {
            std::string line;
            std::cout << "Enter text: ";
            if (!std::getline(std::cin, line)) {
                std::cout << "\nExiting..." << std::endl;
                break;
            }

            memset(g_pBuf, 0, shmSize);
            memcpy(g_pBuf, line.c_str(), line.size() + 1);

            std::cout << "Value written to shared memory: " << line << std::endl;
            if (line == "exit") {
                break;
            }
        }
    } catch (const std::exception& e) {
        ;
    }

    UnmapViewOfFile(g_pBuf);
    CloseHandle(g_hMapFile);

    return 0;
}
