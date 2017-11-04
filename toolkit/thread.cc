
#include "thread.h"
#include "myvector"

#ifndef __TOOLKIT_WXWINDOWS__

void *ThreadRunner(void *userdata) {
  Thread *t = (Thread*) userdata;
  CHECK(t->running_);
  void *retval = t->Entry();
  t->running_ = false;
  if (t->type_ == Thread::DETACHED) {
    delete t;
  }
  return retval;
}

Thread::Thread(ThreadType type) : running_(false), tid_(0), type_(type) {
}

Thread::~Thread() {
  CHECK(!running_);
}

void Thread::Run() {
  CHECK(!running_);
  running_ = true;

  pthread_attr_t detached_attr;
  pthread_attr_init(&detached_attr);
  if (type_ == DETACHED) {
    CHECK(pthread_attr_setdetachstate(&detached_attr,
                                      PTHREAD_CREATE_DETACHED) == 0);
  }
  CHECK(pthread_create(&tid_, (type_ == DETACHED) ? &detached_attr : NULL,
                       &ThreadRunner, this) == 0);
}

void *Thread::Wait() {
  void *retval;
  CHECK(type_ == JOINABLE);
  CHECK(pthread_join(tid_, &retval) == 0);
  return retval;
}

//***************************************************************************
// Parallel for loops.

#include <thread>
#include <mutex>

void ParallelFor(int first, int last, int n, std::function<void(int)> fn) {
  // For a single thread we just call the function directly.
  if (n <= 1) {
    for (int i = first; i <= last; i++) {
      fn(i);
    }
    return;
  }

  // Otherwise we have multiple worker threads call the function.
  std::mutex mutex;             // Protect 'first' and 'last' arguments

  // Worker function.
  auto worker = [&]() {
    // Keep calling fn() until there are no more values left.
    for (;;) {
      int item = 0;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (first <= last) {
          item = first++;
        } else {
          return;
        }
      }
      fn(item);
    }
  };

  // Start worker threads.
  std::vector<std::thread*> threads;
  for (int i = 0; i < n; i++) {
    threads.push_back(new std::thread(worker));
  }

  // Wait for all worker threads to complete.
  for (int i = 0; i < n; i++) {
    threads[i]->join();
    delete threads[i];
  }
}

#endif
