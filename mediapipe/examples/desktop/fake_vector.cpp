
// ZED includes
#include <sl/Camera.hpp>
#include <math.h>       /* isnan, sqrt */

// Sample includes
//#include "GLViewer.hpp"


//Shared memory
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>


//...and for c++ standard library functions:
#include <iostream>
#include <stdexcept>
#include <algorithm>

#ifdef _WIN32
extern "C" { uint32_t GetACP(); }
#endif
#include <cstdlib>


#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>

#include <sstream>
#include <iostream>
#include <stdio.h>


# define NUM_LANDMARKS 21
# define SCALE 2
# define THRESHOLD_CLOSED 24
# define THRESHOLD_ACTIVE_MILIMETERS 700
# define C1 100
# define C2 585
# define C3 75
# define C4 750
# define C5 (C4-(C3*C1)/C1)/(1-C3/C1)
# define C6 (C2-C5)/C1
# define min_count_out 10 
# define COUNT_NOT_HANDS 10 
# define FOCAL_LENGTH 4
constexpr char kInputStream[] = "input_video";
//constexpr char kWindowName[] = "MediaPipe";
constexpr char kOutputLandmarks[] = "hand_landmarks";
constexpr char kSharedMemorySegmentName[] = "SharedMemory";



int main(int argc, char **argv) {
    
    struct shm_remove
   {
      shm_remove() { boost::interprocess::shared_memory_object::remove(kSharedMemorySegmentName); }
      ~shm_remove(){ boost::interprocess::shared_memory_object::remove(kSharedMemorySegmentName); }
   } remover;
    // Create the shared memory segment
    boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, kSharedMemorySegmentName, 65536);

    // Create the vector in shared memory
    typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
    typedef boost::interprocess::vector<int, ShmemAllocator> MyVector;
    MyVector *myvector = segment.construct<MyVector>("MyVector")(segment.get_segment_manager());

  // Create the mutex in shared memory
  typedef boost::interprocess::interprocess_mutex Mutex;
  Mutex *mutex = segment.construct<Mutex>("Mutex")();

  // Create the condition variable in shared memory
  typedef boost::interprocess::interprocess_condition CondVar;
  CondVar *condvar = segment.construct<CondVar>("CondVar")();
    std::cout<<"Shared memory created"<<std::endl;
    // Lock the mutex
    std::unique_lock<Mutex> lock(*mutex);
    // Write the data to shared memory
    myvector->clear();
    
    return EXIT_SUCCESS;
}
