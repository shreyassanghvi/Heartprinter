#include <windows.h>
#include <iostream>
#include <string>
#include <cstring>

int main() {
    const char* shmName = "Local\\PyToCPP";  // Must match Python write shared memory name
    const size_t shmSize = 1024;

    // Open existing shared memory (created by Python) for reading
    HANDLE hMapFile = OpenFileMapping(
        FILE_MAP_READ,   // read-only
        FALSE,
        shmName);

    if (hMapFile == NULL) {
        std::cerr << "OpenFileMapping failed: " << GetLastError() << std::endl;
        return 1;
    }

    void* pBuf = MapViewOfFile(
        hMapFile,
        FILE_MAP_READ,
        0,
        0,
        shmSize);

    if (pBuf == NULL) {
        std::cerr << "MapViewOfFile failed: " << GetLastError() << std::endl;
        CloseHandle(hMapFile);
        return 1;
    }

    std::cout << "Shared memory opened. Reading data from Python:" << std::endl;

    char buffer[shmSize];
    while (true) {
        // Copy data from shared memory - be mindful memory can change asynchronously
        memcpy(buffer, pBuf, shmSize);
        buffer[shmSize - 1] = '\0';  // Ensure null-terminated

        // Print the string (trim at first null)
        std::cout << "\r" << buffer << std::flush;

        Sleep(500);  // 500 ms delay before reading again
    }

    // Cleanup (in practice, never reached because of infinite loop)
    UnmapViewOfFile(pBuf);
    CloseHandle(hMapFile);

    return 0;
}
