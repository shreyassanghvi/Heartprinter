#include <windows.h>
#include <iostream>
#include <cstring>

int main() {
    const char* shmName = "Local\\CPPToPy";
    const size_t shmSize = 1024;

    HANDLE hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,
        NULL,
        PAGE_READWRITE,
        0,
        (DWORD)shmSize,
        shmName);

    if (hMapFile == NULL) {
        std::cerr << "CreateFileMapping failed: " << GetLastError() << std::endl;
        return 1;
    }

    void* pBuf = MapViewOfFile(
        hMapFile,
        FILE_MAP_ALL_ACCESS,
        0,
        0,
        shmSize);

    if (pBuf == NULL) {
        std::cerr << "MapViewOfFile failed: " << GetLastError() << std::endl;
        CloseHandle(hMapFile);
        return 1;
    }

    std::cout << "Shared memory created. Enter integers to write to shared memory counter, Ctrl+C to exit." << std::endl;

    while (true) {
        std::string line;

        std::cout << "Enter text: ";
        if (!std::getline(std::cin, line)) {  // wait for full line input
            std::cout << "\nExiting..." << std::endl;
            break;
        }

        // Zero memory before writing
        memset(pBuf, 0, shmSize);

        // Copy user input including null terminator to shared memory
        memcpy(pBuf, line.c_str(), line.size() + 1);

        std::cout << "Value written to shared memory: " << line << std::endl;
    }

    UnmapViewOfFile(pBuf);
    CloseHandle(hMapFile);

    return 0;
}
