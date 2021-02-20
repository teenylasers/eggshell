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

#include "thread.h"
#include <vector>

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
