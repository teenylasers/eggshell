
#include "thread.h"

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

#endif
