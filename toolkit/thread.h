// Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// Multithreading primitives.

#ifndef __TOOLKIT_THREAD_H__
#define __TOOLKIT_THREAD_H__

#include <functional>
#include <mutex>
#include <condition_variable>
#include "error.h"

#include <pthread.h>

// Threads.
class Thread {
 public:
  // Create a new thread. You must call Join() on joinable threads when they
  // are done. Detached threads always delete themselves when their Main()
  // returns, so they must be allocated on the heap. Detached threads are
  // "fire and forget".
  enum ThreadType { JOINABLE, DETACHED };
  explicit Thread(ThreadType type) : type_(type) {}

  // Once Start() is called this object is only allowed to be destroyed when
  // Entry() returns.
  virtual ~Thread();

  // Start the thread.
  void Run();

  // Do the actual work of this thread. Overridden in subclasses.
  virtual void *Entry() = 0;

  // Join with this thread (only if the thread type is JOINABLE). This waits
  // until the thread exits then returns the value returned by Entry().
  void *Wait();

 private:
  bool running_ = false;
  pthread_t tid_ = 0;
  ThreadType type_;

  friend void *ThreadRunner(void *userdata);
  DISALLOW_COPY_AND_ASSIGN(Thread);
};

// Get the ID of the currently executing thread.
typedef pthread_t CurrentThreadID_t;
inline CurrentThreadID_t GetCurrentThreadID() { return pthread_self(); }

// Ensure a lock is released when a MutexLock goes out of scope.
class MutexLock {
 public:
  explicit MutexLock(std::mutex *mutex) : mu_(mutex) { mu_->lock(); }
  ~MutexLock() { mu_->unlock(); }

 private:
  std::mutex *mu_;
  DISALLOW_COPY_AND_ASSIGN(MutexLock);
};

// Catch bug where variable name is omitted, e.g. MutexLock(&mu);
#define MutexLock(x) COMPILE_ASSERT(0, mutex_lock_decl_missing_var_name)

// Run the function 'fn' from 'n' threads with integer arguments in the range
// [first..last].
void ParallelFor(int first, int last, int n, std::function<void(int)> fn);

#endif
