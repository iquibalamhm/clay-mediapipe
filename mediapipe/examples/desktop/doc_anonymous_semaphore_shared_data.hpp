#include <boost/interprocess/sync/interprocess_semaphore.hpp>

struct shared_memory_buffer2
{
   enum { NumItems = 10 };

   shared_memory_buffer2()
      : mutex(1), nempty(NumItems), nstored(0)
   {}

   //Semaphores to protect and synchronize access
   boost::interprocess::interprocess_semaphore
      mutex, nempty, nstored;

   //Items to fill
   double items[NumItems];
};

struct shared_memory_buffer
{
   enum { NumItems = 10 };

   shared_memory_buffer()
      : mutex(1), nempty(NumItems), nstored(0)
   {}

   //Semaphores to protect and synchronize access
   boost::interprocess::interprocess_semaphore
      mutex, nempty, nstored;

   //Items to fill
   double items[NumItems];
};