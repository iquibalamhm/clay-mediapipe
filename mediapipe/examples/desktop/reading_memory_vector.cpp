#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

using namespace boost::interprocess;

void process2() {
    // Open the shared memory segment
    managed_shared_memory segment(open_only, "SharedMemory");

    // Access the vector in shared memory
    typedef allocator<int, managed_shared_memory::segment_manager> ShmemAllocator;
    typedef vector<int, ShmemAllocator> MyVector;
    MyVector *myvector = segment.find<MyVector>("MyVector").first;

    // Access the mutex in shared memory
    typedef interprocess_mutex Mutex;
    Mutex *mutex = segment.find<Mutex>("Mutex").first;

    // Access the condition variable in shared memory
    typedef interprocess_condition CondVar;
    CondVar *condvar = segment.find<CondVar>("CondVar").first;

    // Process 2 runs at 1000 Hz
    while (true) {
        // Perform other tasks
        // ...
         std::cout << "Doing other stuff..." << std::endl;
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Lock the mutex
        std::unique_lock<Mutex> lock(*mutex);

        // Check if the data has been modified
        if (myvector->size() == 0) {
            lock.unlock();
            continue;
        }

        // Read the data from shared memory
        std::cout << "Process 2: Reading data from shared memory..." << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << (*myvector)[i] << " ";
        }
        std::cout << std::endl;

        // Clear the vector
        myvector->clear();

        // Notify process 1 that the data has been read
        condvar->notify_one();

        // Sleep for 1ms
    }
}

int main() {
    std::thread t2(process2);
    t2.join();

    return 0;
}