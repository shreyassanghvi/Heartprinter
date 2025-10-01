from multiprocessing import shared_memory
import time

def main():
    shm_name = "Local\\PythonSharedMemoryExample"  # must match C++ name
    size = 1024

    # Open existing shared memory block (created by C++)
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
    except FileNotFoundError:
        print("Shared memory not found. Is the C++ program running?")
        return

    print(f"Opened shared memory: {shm.name}, size: {shm.size}")

    # Read null-terminated string from buffer
    buffer = shm.buf[:size]
    message_bytes = bytes(buffer).split(b'\0', 1)[0]
    message = message_bytes.decode('utf-8')

    print(f"Message read from shared memory: '{message}'")
    del buffer
    # Cleanup
    shm.close()

if __name__ == "__main__":
    main()
