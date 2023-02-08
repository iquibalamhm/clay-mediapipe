#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

using namespace boost::interprocess;

void process1() {
    // Create the shared memory segment
    managed_shared_memory segment(create_only, "SharedMemory", 65536);

    // Create the vector in shared memory
    typedef allocator<int, managed_shared_memory::segment_manager> ShmemAllocator;
    typedef vector<int, ShmemAllocator> MyVector;
    MyVector *myvector = segment.construct<MyVector>("MyVector")(segment.get_segment_manager());

    // Create the mutex in shared memory
    typedef interprocess_mutex Mutex;
    Mutex *mutex = segment.construct<Mutex>("Mutex")();

    // Create the condition variable in shared memory
    typedef interprocess_condition CondVar;
    CondVar *condvar = segment.construct<CondVar>("CondVar")();

    // Process 1 runs at 100 Hz
    while (true) {
        // Sleep for 10ms
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
         sleep(0.010);
        // Lock the mutex
        std::unique_lock<Mutex> lock(*mutex);

        // Write the data to shared memory
        myvector->clear();
        for (int i = 0; i < 4; i++) {
            myvector->push_back(rand() % 100);
        }
        std::cout << "Process 1: Writing data to shared memory..." << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << (*myvector)[i] << " ";
        }
        std::cout << std::endl;

        // Notify the condition variable
        condvar->notify_one();
    }
}

int main() {
    std::thread t1(process1);
    t1.detach();

    // Wait for process 2 to run
    //std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}